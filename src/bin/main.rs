#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use core::iter;

    use aligned::{Aligned, A4};
    use stm32f4xx_hal::{
        prelude::*,

        dma::{
            config::{DmaConfig, Priority},
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
            PA4,
            PA6,
            PA7,
            PB0,
            PB2,
            PB10,
            PB12,
            PB13,
            PB15,
            Output,
            PushPull,
        },
        i2s::{
            I2s,
            stm32_i2s_v12x::driver::{*, I2sDriver}, NoMasterClock,
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

    use byte_slice_cast::*;

    use crabdac_firmware::decibels::DECIBELS;
    use crabdac_firmware::pcm1794a;
    use crabdac_firmware::sof_timer::SofTimer;
    use crabdac_firmware::uac::simple_stereo_output::SimpleStereoOutput;

    use fixed::{FixedI32, types::extra::U31};
    type Sample = FixedI32<U31>;

    use ringbuf::{
        Consumer,
        Producer,
        StaticRb,
    };

    const CHANNELS: usize = 2;
    const USB_SAMPLE_SIZE_BYTES: usize = 4;
    const SAMPLE_RATE_HZ: usize = 96000;
    const USB_FRAME_RATE: usize = 1000;
    const MAX_SAMPLES_PER_FRAME: usize = SAMPLE_RATE_HZ / USB_FRAME_RATE + 1;
    const MAX_FRAME_SIZE: usize = CHANNELS * USB_SAMPLE_SIZE_BYTES * MAX_SAMPLES_PER_FRAME;
    const BUFFER_SIZE_WORDS: usize = 512;

    // Pins: (WS, CK, MCK, SD)
    type I2sPins = (PB12, PB13, PA3, PB15);
    //type I2sPins = (PB12, PB13, NoMasterClock, PB15);

    // Pins: (RST, MONO, CHSL, MUTE, FMT0, FMT1)
    type DacPins = (
        PA4<Output<PushPull>>,
        PB10<Output<PushPull>>,
        PB2<Output<PushPull>>,
        PB0<Output<PushPull>>,
        PA7<Output<PushPull>>,
        PA6<Output<PushPull>>
    );

    #[shared]
    struct Shared {
        #[lock_free]
        audio_feedback: u32,
    }

    #[local]
    struct Local {
        dac: pcm1794a::Pcm1794a<DacPins>,
        sof_timer: SofTimer<TIM2, PA0<Alternate<1>>>,

        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_audio: SimpleStereoOutput<'static, UsbBusType>,

        i2s_dma: Transfer<Stream4<DMA1>, 0, I2sDriver<I2s<SPI2, I2sPins>, Master, Transmit, Philips>, MemoryToPeripheral, &'static [u16]>,
        i2s_dma_buffer: &'static mut [u32],

        consumer: Consumer<u32, &'static StaticRb<u32, BUFFER_SIZE_WORDS>>,
        producer: Producer<u32, &'static StaticRb<u32, BUFFER_SIZE_WORDS>>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<TIM5>;

    defmt::timestamp!("{=u32:us}", { monotonics::now().ticks() });

    #[init(local = [
        usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
        usb_ep_memory: [u32; 320] = [0; 320],
        mute_data: [u16; 96] = [0; 96],
        i2s_dma_buffer_memory: Aligned<A4, [u32; 192]> = Aligned([0; 192]),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
                        .use_hse(8.MHz())
                        .sysclk(96.MHz())
                        .hclk(96.MHz())
                        .pclk1(48.MHz())
                        .pclk2(96.MHz())
                        .i2s_clk(98304.kHz()) // 96 kHz * 256 * 4 to generate MCLK
                        //.i2s_ckin(49152.kHz()) // 49.152 MHz = 96 kHz * 256 * 2
                        //.i2s_clk(49152.kHz())
                        .require_pll48clk()
                        .freeze();

        defmt::info!("INIT :: Clocks configured.");
        defmt::info!("INIT ::   HCLK          : {}", clocks.hclk().raw());
        defmt::info!("INIT ::   PCLK1         : {}", clocks.pclk1().raw());
        defmt::info!("INIT ::   PCLK2         : {}", clocks.pclk2().raw());
        defmt::info!("INIT ::   USB (pll48clk): {}", clocks.pll48clk().map(|h| h.raw()));
        defmt::info!("INIT ::   I2S           : {}", clocks.i2s_clk().map(|h| h.raw()));

        let mono = cx.device.TIM5.monotonic_us(&clocks);

        let rb: &mut StaticRb<u32, BUFFER_SIZE_WORDS> = &mut StaticRb::<u32, BUFFER_SIZE_WORDS>::default();
        let (producer, consumer) = unsafe { core::mem::transmute::<&mut StaticRb<u32, BUFFER_SIZE_WORDS>, &'static mut StaticRb<u32, BUFFER_SIZE_WORDS>>(rb) }.split_ref();

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();

        // Enable the I2S oscillator
        //let pa1 = gpioa.pa1.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High);
        let _pa1 = gpioa.pa1.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low);
        // Enable the headphone amplifier opamp
        let _pa8 = gpioa.pa8.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High);

        // (RST, MONO, CHSL, MUTE, FMT0, FMT1)
        let dac_pins = (
            gpioa.pa4.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low),
            gpiob.pb10.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low),
            gpiob.pb2.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low),
            gpiob.pb0.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High),
            gpioa.pa7.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low),
            gpioa.pa6.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::Low),
        );

        let mut dac = pcm1794a::Pcm1794a::new(dac_pins);

        dac.configure(
            pcm1794a::Format::I2S,
            pcm1794a::Channels::Stereo,
            pcm1794a::DfRolloff::Sharp
        ).unwrap();

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
            SAMPLE_RATE_HZ as u32,
            USB_SAMPLE_SIZE_BYTES,
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
            .request_frequency(SAMPLE_RATE_HZ as u32);
        let mut i2s_driver = I2sDriver::new(i2s, i2s_config);
        i2s_driver.set_tx_dma(true);
        i2s_driver.set_tx_interrupt(false);
        defmt::info!("INIT :: i2s sample rate {}", i2s_driver.sample_rate());

        // LOOOOL 🦀
        // Grabs a pointer to the static buffer, manipulating the type system so
        // the Transfer knows we're sending the right number of half-words
        let ptr = unsafe {
            core::mem::transmute::<&[u16], &'static [u16]>(
                cx.local.i2s_dma_buffer_memory.as_byte_slice().as_slice_of::<u16>().unwrap()
            )
        };
        let dma1_stream4 = StreamsTuple::new(cx.device.DMA1).4;
        let mut i2s_dma = Transfer::init_memory_to_peripheral(
            dma1_stream4,
            i2s_driver,
            ptr,
            Some(ptr),
            DmaConfig::default()
                .double_buffer(true)
                .priority(Priority::VeryHigh)
                .memory_increment(true)
                .peripheral_increment(false)
                .transfer_error_interrupt(true)
                .half_transfer_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        i2s_dma.start(|i2s| {
            i2s.enable();
            dac.enable().unwrap();
        });

        (
            Shared {
                audio_feedback: (SAMPLE_RATE_HZ as u32 * 256 / 1000) << 6,
            },
            Local {
                dac,
                sof_timer,
                usb_dev,
                usb_audio,
                i2s_dma,
                i2s_dma_buffer: cx.local.i2s_dma_buffer_memory.as_mut_slice(),
                producer,
                consumer,
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

    #[task(binds = TIM2, priority = 1, local = [sof_timer, clocks: u32 = 0, sof_count: u8 = 0], shared = [audio_feedback])]
    fn sof_timer_handler(cx: sof_timer_handler::Context) {
        let period = cx.local.sof_timer.get_period_clocks();

        if *cx.local.sof_count == 3 {
            let clocks = period + *cx.local.clocks;
            *cx.shared.audio_feedback = clocks << 4;
            defmt::debug!("SOF :: {0=14..24} + {0=0..14}/16384", clocks << 4);
            *cx.local.clocks = 0;
            *cx.local.sof_count = 0;
        } else {
            *cx.local.clocks += period;
            *cx.local.sof_count += 1;
        }
    }

    #[task(binds = OTG_FS, priority = 1, local = [
        usb_dev,
        usb_audio,
        producer,
        dac,
        buf: [u8; MAX_FRAME_SIZE] = [0; MAX_FRAME_SIZE],
    ],
    shared = [audio_feedback])]
    fn usb_handler(cx: usb_handler::Context) {
        let start = monotonics::now();
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.local.usb_dev;
        let usb_audio: &mut SimpleStereoOutput<UsbBusType> = cx.local.usb_audio;

        if usb_dev.poll(&mut [usb_audio]) {
            cx.local.dac.set_mute(usb_audio.mute).unwrap();

            if usb_audio.audio_feedback_needed {
                match usb_audio.write_raw_feedback(*cx.shared.audio_feedback) {
                    Ok(_) => defmt::debug!("USB :: Feedback OK {0=14..24}.{0=0..14}", *cx.shared.audio_feedback),
                    Err(_) => defmt::warn!("USB :: Feedback ERR"),
                }
            }

            if usb_audio.audio_data_available {
                let bytes_received = usb_audio.read_audio_data(cx.local.buf).unwrap();
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
                if cx.local.producer.free_len() >= bytes_received / 4 {
                    if usb_audio.mute {
                        cx.local.producer.push_iter(&mut iter::repeat(0u32).take(bytes_received / 4));
                    } else {
                        let volume_multiplier = Sample::from_bits(DECIBELS[((usb_audio.volume >> 8) + 127) as usize]);

                        cx.local.producer.push_iter(
                            &mut cx.local.buf[0..bytes_received].as_slice_of::<i32>().unwrap().iter()
                                .map(|s| Sample::from_bits(*s))
                                .map(|s| s * volume_multiplier)
                                .map(|s| s.to_bits() as u32)
                                .map(|s| s << 16 | s >> 16)
                        );

                    }
                }
            }
        }
        defmt::debug!("usb_handler: {}", monotonics::now() - start);
    }

    #[task(binds = DMA1_STREAM4, priority = 3, local = [
        i2s_dma,
        i2s_dma_buffer,
        consumer,
        last: Option<Instant<u32, 1, 1000000>> = None,
    ])]
    fn i2s_dma_handler(cx: i2s_dma_handler::Context) {
        let i2s_dma = cx.local.i2s_dma;
        let half_len = cx.local.i2s_dma_buffer.len() / 2;

        if Stream4::<DMA1>::get_transfer_error_flag() {
            defmt::warn!("i2s_dma :: transfer error");
            i2s_dma.clear_transfer_error_interrupt();
        }

        if Stream4::<DMA1>::get_half_transfer_flag() {
            i2s_dma.clear_half_transfer_interrupt();

            if cx.local.consumer.len() >= half_len {
                for i in 0..half_len {
                    cx.local.i2s_dma_buffer[i] = cx.local.consumer.pop().unwrap();
                }
            }
        }

        if Stream4::<DMA1>::get_transfer_complete_flag() {
            i2s_dma.clear_transfer_complete_interrupt();

            if cx.local.consumer.len() >= half_len {
                for i in 0..half_len {
                    cx.local.i2s_dma_buffer[i+half_len] = cx.local.consumer.pop().unwrap();
                }
            }
        }


        match cx.local.last {
            None => *cx.local.last = Some(monotonics::now()),
            Some(last) => {
                let delay = monotonics::now() - *last;
                if delay > 1400.micros::<1, 1000000>() {
                    defmt::warn!("i2s_dma :: delayed ({})", delay);
                }
                cx.local.last.replace(monotonics::now());
            }
        }

    }
}
