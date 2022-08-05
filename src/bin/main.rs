#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use stm32f4xx_hal::{
        prelude::*,

        dma::{
            config::{DmaConfig, Priority, BurstMode},
            traits::StreamISR,
            MemoryToPeripheral,
            Stream4,
            StreamsTuple,
            Transfer,
        },
        gpio::{
            Alternate,
            PA0,
            PA3,
            PB12,
            PB13,
            PB15,
        },
        i2s::{
            I2s,
            stm32_i2s_v12x::driver::{*, I2sDriver},
        },
        pac::{
            DMA1,
            SPI2,
            TIM2,
            TIM5,
        },
        otg_fs::{
            USB,
            UsbBus,
            UsbBusType,
        },
        timer::{
            fugit::{
                Instant,
                ExtU32,
            },
            MonoTimerUs
        },
    };

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use bbqueue::{
        BBBuffer,
        Consumer,
        Producer,
        GrantR,
    };

    use byte_slice_cast::*;

    use crabdac_firmware::sof_timer::SofTimer;
    use crabdac_firmware::uac::simple_stereo_output::SimpleStereoOutput;

    const CHANNELS: usize = 2;
    const USB_SAMPLE_SIZE: usize = 4;
    const SAMPLE_RATE: usize = 96000;
    const USB_FRAME_RATE: usize = 1000;
    const MAX_SAMPLES_PER_FRAME: usize = SAMPLE_RATE / USB_FRAME_RATE + 1;
    const MAX_FRAME_SIZE: usize = CHANNELS * USB_SAMPLE_SIZE * MAX_SAMPLES_PER_FRAME;
    const BUFFER_SIZE: usize = MAX_FRAME_SIZE * 8;

    #[shared]
    struct Shared {
        audio_feedback: u32,
        buffered_byte_count: usize,
    }

    #[local]
    struct Local {
        producer: Producer<'static, BUFFER_SIZE>,
        consumer: Consumer<'static, BUFFER_SIZE>,
        sof_timer: SofTimer<TIM2, PA0<Alternate<1>>>,

        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_audio: SimpleStereoOutput<'static, UsbBusType>,

        i2s_dma: Transfer<Stream4<DMA1>, 0, I2sDriver<I2s<SPI2, (PB12, PB13, PA3, PB15)>, Master, Transmit, Philips>, MemoryToPeripheral, &'static [u16]>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<TIM5>;

    defmt::timestamp!("{=u32:us}", { monotonics::now().ticks() });

    #[init(local = [
        #[link_section = ".data.buffer"]
        buffer: BBBuffer<BUFFER_SIZE> = BBBuffer::new(),
        usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
        usb_ep_memory: [u32; 320] = [0; 320],
        mute_data: [u16; 96] = [0; 96],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
                        .use_hse(25.MHz())
                        .sysclk(96.MHz())
                        .hclk(96.MHz())
                        .pclk1(48.MHz())
                        .pclk2(96.MHz())
                        .i2s_clk(98304.kHz()) // 96 kHz * 256 * 4 to generate MCLK
                        //.i2s_ckin(49152.kHz()) // 49.152 MHz = 96 kHz * 256 * 2
                        .require_pll48clk()
                        .freeze();

        defmt::info!("INIT :: Clocks configured.");
        defmt::info!("INIT ::   HCLK          : {}", clocks.hclk().raw());
        defmt::info!("INIT ::   PCLK1         : {}", clocks.pclk1().raw());
        defmt::info!("INIT ::   PCLK2         : {}", clocks.pclk2().raw());
        defmt::info!("INIT ::   USB (pll48clk): {}", clocks.pll48clk().map(|h| h.raw()));
        defmt::info!("INIT ::   I2S           : {}", clocks.i2s_clk().map(|h| h.raw()));

        let mono = cx.device.TIM5.monotonic_us(&clocks);

        let (producer, consumer) = cx.local.buffer.try_split().unwrap();

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();

        let tim2_etr = gpioa.pa0.into_alternate();
        let sof_timer = SofTimer::new(cx.device.TIM2, tim2_etr);

        let usb = USB {
            usb_global: cx.device.OTG_FS_GLOBAL,
            usb_device: cx.device.OTG_FS_DEVICE,
            usb_pwrclk: cx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        *cx.local.usb_bus = Some(UsbBus::new(usb, cx.local.usb_ep_memory));

        let usb_audio = SimpleStereoOutput::new(
            cx.local.usb_bus.as_ref().unwrap(),
            96000,
            4,
            24
        );

        let usb_dev = UsbDeviceBuilder::new(cx.local.usb_bus.as_ref().unwrap(), UsbVidPid(0x1209, 0x0001))
            .manufacturer("Crabs Pty Ltd.")
            .product("CrabDAC")
            .serial_number("420.69")
            .composite_with_iads()
            .max_packet_size_0(8)
            .self_powered(true)
            .build();

        let i2s_pins = (gpiob.pb12, gpiob.pb13, gpioa.pa3, gpiob.pb15);
        let i2s = I2s::new(cx.device.SPI2, i2s_pins, &clocks);
        let i2s_config = I2sDriverConfig::new_master()
            .transmit()
            .standard(Philips)
            .data_format(DataFormat::Data24Channel32)
            .master_clock(true)
            .request_frequency(SAMPLE_RATE as u32);
        let mut i2s_driver = I2sDriver::new(i2s, i2s_config);
        i2s_driver.set_tx_dma(true);
        i2s_driver.set_tx_interrupt(true);
        defmt::info!("INIT :: i2s sample rate {}", i2s_driver.sample_rate());

        let dma1_stream4 = StreamsTuple::new(cx.device.DMA1).4;
        let mut i2s_dma = Transfer::init_memory_to_peripheral(
            dma1_stream4,
            i2s_driver,
            (*cx.local.mute_data).as_slice(),
            None,
            DmaConfig::default()
                .priority(Priority::High)
                .memory_increment(true)
                .peripheral_increment(false)
                .transfer_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        i2s_dma.start(|i2s| i2s.enable());

        (
            Shared {
                audio_feedback: (SAMPLE_RATE as u32 * 256 / 1000) << 6,
                buffered_byte_count: 0,
            },
            Local {
                producer,
                consumer,
                sof_timer,
                usb_dev,
                usb_audio,
                i2s_dma,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    #[inline(never)]
    #[link_section = ".data.sof_timer_handler"]
    #[task(binds = TIM2, priority = 2, local = [sof_timer, clocks: u32 = 0, sof_count: u8 = 0], shared = [audio_feedback])]
    fn sof_timer_handler(mut cx: sof_timer_handler::Context) {
        let period = cx.local.sof_timer.get_period_clocks();

        if *cx.local.sof_count == 3 {
            let clocks = period + *cx.local.clocks;
            cx.shared.audio_feedback.lock(|x| *x = clocks << 4);
            defmt::debug!("SOF :: {0=14..24} + {0=0..14}/16384", clocks << 4);
            *cx.local.clocks = 0;
            *cx.local.sof_count = 0;
        } else {
            *cx.local.clocks += period;
            *cx.local.sof_count += 1;
        }
    }

    #[task(binds = OTG_FS, priority = 1, local = [
        producer,
        usb_dev,
        usb_audio,
        rx_bytes: usize = 0,
        rx_rate_timestamp: Option<Instant<u32, 1, 1000000>> = None,
    ],
    shared = [audio_feedback, buffered_byte_count])]
    fn usb_handler(mut cx: usb_handler::Context) {
        let start = monotonics::now();
        let producer: &mut bbqueue::Producer<'static, BUFFER_SIZE> = cx.local.producer;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.local.usb_dev;
        let usb_audio: &mut SimpleStereoOutput<UsbBusType> = cx.local.usb_audio;

        usb_dev.poll(&mut [usb_audio]);
        if usb_audio.audio_feedback_needed {
            cx.shared.audio_feedback.lock(|x| {
                match usb_audio.write_raw_feedback(*x) {
                    Ok(_) => defmt::debug!("USB :: Feedback OK {0=14..24}.{0=0..14}", *x),
                    Err(_) => defmt::warn!("USB :: Feedback ERR"),
                }
            })
        }

        if usb_audio.audio_data_available {
            let mut grant = producer.grant_exact(MAX_FRAME_SIZE).unwrap();
            defmt::debug!("WG: {:#06x} +{:#05x}", grant.buf().as_ptr() as usize - 0x2001e690, grant.len());
            let bytes_received = usb_audio.read_audio_data(&mut *grant).unwrap();
            if bytes_received < (96 * 4 * 2) {
                defmt::debug!("usb_handler :: {} samples received", bytes_received / 4 / 2);
            }

            // USB PCM data is 24-bit left-justified in a 32-bit value, where
            // the MSB is the sign bit.
            //
            // I2S Philips format expects to send two half-words, with the
            // most-significant transmitted _first_. To do so, we need to
            // reverse the memory order of the half-words from the USB format.
            //
            // e.g. to transmit the 24-bit value 0x8EAA33 over i2s:
            //
            // usb receives [0x00, 0x33, 0xaa, 0x8e]
            // interpreted as a 32-bit little-endian value: 0x8eaa3300
            //
            // swap half-words, giving 0x33008eaa
            // represented in memory as [0xaa, 0x8e, 0x00, 0x33]
            //
            // i2s transmits the first half-word, from memory [0xaa, 0x8e] = 0x8eaa
            // i2s transmits the other half-word, from memory [0x00, 0x33] = 0x3300
            for w in grant.as_mut_slice_of::<u32>().unwrap().iter_mut() {
                // This should compile as a single REV instruction on arm
                *w = *w << 16 | *w >> 16;
            }
            grant.commit(bytes_received);
            defmt::debug!("WC:        +{:#05x}", bytes_received);
            defmt::debug!("USB :: received {} bytes of audio data", bytes_received);
            let buffered_byte_count = cx.shared.buffered_byte_count.lock(|x| {*x += bytes_received; *x});

            match *cx.local.rx_rate_timestamp {
                None => {
                    *cx.local.rx_rate_timestamp = Some(monotonics::now());
                    *cx.local.rx_bytes = 0;
                },
                Some(last) => {
                    *cx.local.rx_bytes += bytes_received;
                    let micros = monotonics::now() - last;
                    if micros >= 1.secs::<1, 1000000>() {
                        let rate = *cx.local.rx_bytes as u32 * (micros.ticks() / 1000) / 1000;
                        defmt::info!("USB RX Rate: {}", rate);
                        defmt::info!("Buffered: {:#x}", buffered_byte_count);
                        *cx.local.rx_bytes = 0;
                        cx.local.rx_rate_timestamp.replace(monotonics::now());
                    }
                }
            }
        }
        defmt::debug!("usb_handler: {}", monotonics::now() - start);
    }

    #[inline(never)]
    #[link_section = ".data.i2s_dma_handler"]
    #[task(binds = DMA1_STREAM4, priority = 3, local = [
        i2s_dma,
        consumer,
        active_grant: Option<GrantR<'static, BUFFER_SIZE>> = None,
    ],
    shared = [buffered_byte_count])]
    fn i2s_dma_handler(mut cx: i2s_dma_handler::Context) {
        let start = monotonics::now();
        let active_grant = cx.local.active_grant;
        let consumer = cx.local.consumer;
        let i2s_dma = cx.local.i2s_dma;

        if Stream4::<DMA1>::get_transfer_error_flag() {
            defmt::warn!("i2s_dma :: transfer error");
            i2s_dma.clear_transfer_error_interrupt();
        }

        if Stream4::<DMA1>::get_transfer_complete_flag() {
            i2s_dma.clear_transfer_complete_interrupt();

            match active_grant.take() {
                Some(g) => drop(g),
                None => {},
            }

            match consumer.read() {
                Err(_) => {
                    defmt::debug!("i2s_dma :: Failed to read grant");
                    unsafe { i2s_dma.next_transfer_with(|buf, _| (buf, buf)).unwrap(); }
                    *active_grant = None;
                },
                Ok(mut grant) => {
                    cx.shared.buffered_byte_count.lock(|x| *x -= grant.len());
                    defmt::debug!("RG:                  {:#06x} +{:#05x}", grant.buf().as_ptr() as usize - 0x2001e690, grant.len());
                    let words: &'static [u16] = unsafe { grant.as_static_buf() }.as_slice_of::<u16>().unwrap();
                    defmt::debug!("i2s_dma :: Writing {} half-words", words.len());
                    i2s_dma.next_transfer(words).unwrap();
                    let now = monotonics::now();
                    let delay = now - start;
                    if delay > 5.micros::<1, 1000000>() { defmt::info!("i2s_dma :: {} delay", delay); }
                    grant.to_release(grant.len());
                    active_grant.replace(grant);
                }
            }
        }

    }

    #[task(binds = SPI2, priority = 1)]
    fn i2s_handler(_cx: i2s_handler::Context) {
        let spi2 = unsafe { &(*SPI2::ptr()) };
        defmt::debug!("I2S TXE: {}", spi2.sr.read().txe().bit());
        if spi2.sr.read().udr().bit_is_set() { defmt::warn!("I2S :: Underrurn") }

    }
}
