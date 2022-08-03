#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use core::intrinsics::transmute;

    use stm32f4xx_hal::{
        prelude::*,

        dma::{
            traits::StreamISR,
            Stream4,
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
        timer::{MonoTimerUs, fugit::ExtU32}, dma::{StreamsTuple, Transfer, config::{DmaConfig, Priority, BurstMode}, MemoryToPeripheral},
    };

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use aligned::{Aligned, A4};

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
    const BUFFER_SIZE: usize = MAX_FRAME_SIZE * 3;

    #[shared]
    struct Shared {
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

    #[init(local = [
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
                        .i2s_clk(98285714.Hz()) // Close to 98.304 MHz, -0.01% error
                        //.i2s_ckin(49152.kHz()) // 49.152 MHz = 96 kHz * 256 * 2
                        .require_pll48clk()
                        .freeze();

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
        defmt::info!("init :: i2s sample rate {}", i2s_driver.sample_rate());

        let dma1_stream4 = StreamsTuple::new(cx.device.DMA1).4;
        let mut i2s_dma = Transfer::init_memory_to_peripheral(
            dma1_stream4,
            i2s_driver,
            (*cx.local.mute_data).as_slice(),
            None,
            DmaConfig::default()
                //.priority(Priority::High)
                //.double_buffer(false)
                .memory_burst(BurstMode::Burst16)
                .memory_increment(true)
                .peripheral_increment(false)
                .transfer_complete_interrupt(true),
        );
        i2s_dma.start(|i2s| i2s.enable());

        (
            Shared {
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
    #[task(binds = TIM2, priority = 3, local = [sof_timer])]
    fn sof_timer_handler(cx: sof_timer_handler::Context) {
        let period = cx.local.sof_timer.get_period_clocks();
        defmt::debug!("SOF :: {0=14..24}.{0=0..14}", period);
    }

    #[inline(never)]
    #[link_section = ".data.usb_handler"]
    #[task(binds = OTG_FS, priority = 2, local = [
        producer,
        usb_dev,
        usb_audio,
        usb_audio_buf: Aligned<A4, [u8; MAX_FRAME_SIZE]> = Aligned([0; MAX_FRAME_SIZE])
    ])]
    fn usb_handler(cx: usb_handler::Context) {
        let producer: &mut bbqueue::Producer<'static, BUFFER_SIZE> = cx.local.producer;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.local.usb_dev;
        let usb_audio: &mut SimpleStereoOutput<UsbBusType> = cx.local.usb_audio;

        usb_dev.poll(&mut [usb_audio]);
        if usb_audio.audio_feedback_needed {
            match usb_audio.write_raw_feedback(24576 << 6) {
                Ok(_) => defmt::debug!("USB :: Feedback OK"),
                Err(_) => defmt::warn!("USB :: Feedback ERR"),
            }
        }

        if usb_audio.audio_data_available {
            let mut grant = producer.grant_exact(MAX_FRAME_SIZE).unwrap();
            let bytes_received = usb_audio.read_audio_data(&mut *grant).unwrap();
            for w in grant.as_mut_slice_of::<u32>().unwrap().iter_mut() {
                *w = *w << 16 | *w >> 16;
            }
            grant.commit(bytes_received);
            defmt::debug!("USB :: received {} bytes of audio data", bytes_received);
            //fake_i2s::spawn_after(500.micros()).unwrap();
        }
    }

    // #[task(priority = 3, local = [consumer, count: usize = 0])]
    // fn fake_i2s(cx: fake_i2s::Context) {
    //     let consumer = cx.local.consumer;

    //     match consumer.read() {
    //         Ok(grant) => {
    //             let len = grant.len();
    //             defmt::debug!("fake_i2s :: 0x{:x} bytes", len);
    //             if *cx.local.count % 1000 == 0 {
    //                 defmt::info!("{=[?]:04x}", grant.as_slice_of::<u16>().unwrap());
    //             }
    //             *cx.local.count += 1;
    //             grant.release(len);
    //         },
    //         Err(_) => defmt::error!("fake_i2s :: consumer.read() failed"),
    //     }
    // }

    #[task(binds = DMA1_STREAM4, priority = 3, local = [
        i2s_dma,
        consumer,
        active_grant: Option<GrantR<'static, BUFFER_SIZE>> = None,
        count: usize = 0,
        mute_data: [u16; 96] = [0; 96],
    ])]
    fn i2s_dma_handler(cx: i2s_dma_handler::Context) {
        let active_grant = cx.local.active_grant;
        let consumer = cx.local.consumer;
        let i2s_dma = cx.local.i2s_dma;
        let mute_data = *cx.local.mute_data;

        if Stream4::<DMA1>::get_fifo_error_flag() {
            //defmt::warn!("i2s_dma :: fifo error");
            i2s_dma.clear_fifo_error_interrupt();
        }

        if Stream4::<DMA1>::get_transfer_complete_flag() {
            //defmt::info!("i2s_dma :: transfer complete");
            i2s_dma.clear_transfer_complete_interrupt();

            match active_grant.take() {
                Some(g) => {
                    defmt::debug!("Releasing 192 bytes");
                    g.release(32)
                },
                None => {},
            }

            match consumer.read() {
                Err(_) => {
                    defmt::debug!("i2s_dma :: Failed to read grant");
                    let buf = unsafe { transmute::<&[u16], &'static [u16]>(&mute_data) };
                    i2s_dma.next_transfer(buf).unwrap();
                },
                Ok(grant) => {
                    let words: &'static [u16] = unsafe { &grant.as_static_buf()[0..16] }.as_slice_of::<u16>().unwrap();
                    defmt::debug!("i2s_dma :: Writing {} half-words", words.len());
                    active_grant.replace(grant);
                    i2s_dma.next_transfer(words).unwrap();
                }
            }
        }
    }
}
