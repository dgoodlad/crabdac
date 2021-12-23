#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {
    use core::intrinsics::transmute;

    use crabdac::timer::CaptureChannel;
    use crabdac::timer::UsbAudioFrequencyFeedback;
    use crabdac::uac;
    use crabdac::uac::StreamingState;
    use hal::dma::StreamX;
    use hal::gpio::gpiob::PB8;

    use bbqueue;
    use bytemuck::*;
    use crabdac::uac::UsbAudioClass;
    use hal::gpio::Input;
    use stm32_i2s_v12x::MasterConfig;
    use stm32_i2s_v12x::Polarity;
    use stm32_i2s_v12x::TransmitMode;
    use stm32_i2s_v12x::format::Data24Frame32;
    use stm32_i2s_v12x::format::FrameFormat;
    use stm32f4xx_hal::dma::traits::StreamISR;
    use stm32f4xx_hal::dma::MemoryToPeripheral;
    use stm32f4xx_hal::dma::Stream5;
    use stm32f4xx_hal::dma::StreamsTuple;
    use stm32f4xx_hal::dma::Transfer;
    use stm32f4xx_hal::dma::config::DmaConfig;
    use stm32f4xx_hal::gpio::Alternate;
    use stm32f4xx_hal::gpio::PushPull;
    use stm32f4xx_hal::gpio::gpioa::PA4;
    use stm32f4xx_hal::gpio::gpioa::PA15;
    use stm32f4xx_hal::gpio::gpioc::PC7;
    use stm32f4xx_hal::gpio::gpioc::PC10;
    use stm32f4xx_hal::gpio::gpioc::PC12;
    use stm32f4xx_hal::i2s;
    use stm32f4xx_hal::{prelude::*, pac, timer::{monotonic::MonoTimer, Timer}};
    use stm32f4xx_hal::rcc::BusTimerClock;

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use stm32f4xx_hal as hal;
    use hal::otg_hs::{USB, UsbBus, UsbBusType};

    const CHANNELS: u32 = 2;
    const SAMPLE_RATE: u32 = 96000;
    const USB_FRAME_RATE: u32 = 1000;
    const SAMPLE_WORD_SIZE: usize = 4;
    const MAX_FRAME_SIZE: usize = ((SAMPLE_RATE / USB_FRAME_RATE + 1) * CHANNELS) as usize * SAMPLE_WORD_SIZE;
    const FRAME_HEADER_SIZE: usize = 2;
    const BUFFER_SIZE: usize = (MAX_FRAME_SIZE + FRAME_HEADER_SIZE) * 4;

    #[shared]
    struct Shared {
        i2s_dma: Option<I2sDmaTransfer>,
        feedback_clock_counter: uac::ClockCounter,
    }

    #[local]
    struct Local {
        producer: bbqueue::framed::FrameProducer<'static, BUFFER_SIZE>,
        consumer: bbqueue::framed::FrameConsumer<'static, BUFFER_SIZE>,
        write_grant: Option<bbqueue::framed::FrameGrantW<'static, BUFFER_SIZE>>,
        read_grant: Option<bbqueue::framed::FrameGrantR<'static, BUFFER_SIZE>>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_audio: UsbAudioClass<'static, UsbBusType>,
        dma1_stream_5: Option<StreamX<pac::DMA1, 5>>,
        i2s: Option<I2sDevice>,
        feedback_timer: UsbAudioFrequencyFeedback<pac::TIM2, PB8<Alternate<PushPull, 1>>>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [audio_queue: bbqueue::BBBuffer<BUFFER_SIZE> = bbqueue::BBBuffer::new(),
                    zeroes: [u16; 768] = [0; 768],
                    usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
                    ep_memory: [u32; 320] = [0; 320],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init: clocks");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(180.mhz())
            .hclk(180.mhz())
            .pclk1(45.mhz())
            .pclk2(90.mhz())
            .i2s_apb1_clk(98400.khz())
            .require_pll48clk()
            .freeze();

        assert!(clocks.is_pll48clk_valid());

        defmt::info!("clocks.hclk: {}", clocks.hclk().0);
        defmt::info!("clocks.pclk1: {}", clocks.pclk1().0);
        defmt::info!("clocks.pclk2: {}", clocks.pclk2().0);
        defmt::info!("clocks.pll48clk: {}", clocks.pll48clk().unwrap().0);
        defmt::info!("clocks.i2s_apb1_clk: {}", clocks.i2s_apb1_clk().unwrap().0);
        defmt::info!("TIM2 clock: {}", pac::TIM2::timer_clock(&clocks).0);

        defmt::info!("init: monotonic timer");

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

        let (producer, consumer) = cx.local.audio_queue.try_split_framed().unwrap();

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();

        // OTG_VBUS_SENSE
        let pb13: hal::gpio::Pin<Input<hal::gpio::Floating>, 'B', 13> = gpiob.pb13.into_floating_input();

        // OTG_FS_SOF
        let pa4: PA4<Alternate<PushPull, 12>> = gpioa.pa4.into_alternate();

        // TIM2_ETR
        let pb8: PB8<Alternate<PushPull, 1>> = gpiob.pb8.into_alternate();

        let feedback_timer: UsbAudioFrequencyFeedback<pac::TIM2, PB8<Alternate<PushPull, 1>>> =
            UsbAudioFrequencyFeedback::new(cx.device.TIM2, CaptureChannel::Channel1, pb8);
        feedback_timer.start();


        let usb = USB {
            usb_global: cx.device.OTG_HS_GLOBAL,
            usb_device: cx.device.OTG_HS_DEVICE,
            usb_pwrclk: cx.device.OTG_HS_PWRCLK,
            pin_dm: gpiob.pb14.into_alternate(),
            pin_dp: gpiob.pb15.into_alternate(),
            hclk: clocks.hclk(),
        };

        *cx.local.usb_bus = Some(UsbBus::new(usb, cx.local.ep_memory));

        let usb_audio = UsbAudioClass::new(cx.local.usb_bus.as_ref().unwrap());

        defmt::debug!("USB Audio device created");

        let usb_dev = UsbDeviceBuilder::new(cx.local.usb_bus.as_ref().unwrap(), UsbVidPid(0x1209, 0x0001))
            .manufacturer("Crabs Pty Ltd.")
            .product("CrabDAC")
            .serial_number("TEST")
            // This class/subclass/protocol indicates that there is an Interface
            // Association Descriptor included in the configuration.
            .device_class(0xef)
            .device_sub_class(0x02)
            .device_protocol(0x01)
            .max_packet_size_0(8)
            .self_powered(true)
            .max_power(250)
            .build();

        defmt::debug!("USB Device created");

        let i2s_periph = I2sPeripheral::new(
            cx.device.SPI3, (
                gpioa.pa15.into_alternate(),
                gpioc.pc10.into_alternate(),
                gpioc.pc7.into_alternate(),
                gpioc.pc12.into_alternate(),
            ),
            &clocks
        );

        let i2s_clock = i2s_periph.input_clock();

        let i2s_config = MasterConfig::with_sample_rate(
            i2s_clock.0,
            SAMPLE_RATE,
            Data24Frame32,
            FrameFormat::PhilipsI2s,
            Polarity::IdleHigh,
            stm32_i2s_v12x::MasterClock::Enable
        );

        let mut i2s: I2sDevice = stm32_i2s_v12x::I2s::new(i2s_periph)
            .configure_master_transmit(i2s_config);
        i2s.set_dma_enabled(true);

        let dma1_streams = StreamsTuple::new(cx.device.DMA1);

        let feedback_clock_counter = uac::ClockCounter::new(8);

        (
            Shared {
                i2s_dma: None,
                feedback_clock_counter,
            },
            Local {
                producer,
                consumer,
                read_grant: None,
                write_grant: None,
                usb_dev,
                usb_audio,
                dma1_stream_5: Some(dma1_streams.5),
                i2s: Some(i2s),
                feedback_timer,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [i2s_dma], local = [dma1_stream_5, i2s])]
    fn toggle_i2s_dma(mut cx: toggle_i2s_dma::Context, enable: uac::StreamingState) {
        static ZEROES: [u16; 96 * 2] = [0; 96 * 2];
        let toggle_i2s_dma::LocalResources { dma1_stream_5, i2s } = cx.local;

        if enable == StreamingState::Enabled {
            assert!(dma1_stream_5.is_some());
            assert!(i2s.is_some());

            let dma_config: DmaConfig = DmaConfig::default()
                .double_buffer(false)
                .fifo_enable(true)
                .fifo_threshold(stm32f4xx_hal::dma::config::FifoThreshold::HalfFull)
                .priority(stm32f4xx_hal::dma::config::Priority::VeryHigh)
                .transfer_complete_interrupt(true);

            let mut i2s_dma: I2sDmaTransfer =
                Transfer::init_memory_to_peripheral(dma1_stream_5.take().unwrap(), i2s.take().unwrap(), &ZEROES, None, dma_config);

            i2s_dma.start(|i2s| {
                defmt::info!("Started I2S DMA stream");
                i2s.enable();
            });

            cx.shared.i2s_dma.lock(|o| o.replace(i2s_dma));
        } else {
            //assert!(dma1_stream_5.is_none());
            //assert!(i2s.is_none());

            cx.shared.i2s_dma.lock(|x| {
                match x.take() {
                    Some(i2s_dma) => {
                        let (stream, peripheral, _buf, _) = i2s_dma.release();
                        dma1_stream_5.replace(stream);
                        i2s.replace(peripheral);
                        // TODO release the buffer grant somehow
                    },
                    None => {}
                }
            });
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    #[task(binds = OTG_HS, local = [producer, usb_dev, usb_audio], shared = [feedback_clock_counter])]
    fn usb_handler(mut cx: usb_handler::Context) {
        let producer = cx.local.producer;
        let usb_dev = cx.local.usb_dev;
        let usb_audio: &mut UsbAudioClass<UsbBusType> = cx.local.usb_audio;

        while usb_dev.poll(&mut [usb_audio]) {
            if usb_audio.audio_data_available {
                defmt::trace!("usb_handler :: requesting frame up to {:#x} bytes", MAX_FRAME_SIZE);
                let mut grant = match producer.grant(MAX_FRAME_SIZE) {
                    Ok(grant) => grant,
                    Err(_) => { defmt::info!("Dropped USB Frame"); continue; }
                };

                let bytes_received = usb_audio.read_audio_stream(&mut grant).unwrap();
                grant.commit(bytes_received);

                defmt::trace!("usb_handler :: committed {:#x} bytes", bytes_received);
            }

            if usb_audio.audio_feedback_needed {
                cx.shared.feedback_clock_counter.lock(|counter| {
                    usb_audio.write_audio_feedback(counter).unwrap_or_else(|_e| {
                        defmt::debug!("usb_handler :: feedback skipped, would block");
                        0
                    });
                    counter.clear();
                });
            }

            usb_audio.enable_disable.take().map(|b| {
                if b == StreamingState::Enabled {
                    usb_audio.audio_feedback_needed = true;
                }
                match toggle_i2s_dma::spawn(b) {
                    Ok(_) => defmt::info!("Toggled I2S DMA"),
                    Err(_) => defmt::info!("Failed to toggle I2s DMA")
                }
            });
        }
    }

    #[task(binds = DMA1_STREAM5, priority = 3, local = [consumer, read_grant], shared = [i2s_dma])]
    fn i2s_dma_handler(cx: i2s_dma_handler::Context) {
        static ZEROES: [u16; 96 * 2] = [0; 96 * 2];

        let i2s_dma_handler::Context { local, shared } = cx;
        let i2s_dma_handler::LocalResources { consumer, read_grant } = local;
        let i2s_dma_handler::SharedResources { mut i2s_dma } = shared;

        defmt::debug!("i2s_dma_handler");

        if Stream5::<pac::DMA1>::get_transfer_complete_flag() {
            defmt::debug!("i2s_dma_handler :: transfer complete");

            read_grant.take().map(|g| {
                let len = g.len();
                defmt::debug!("i2s dma handler :: released {:#x} bytes", len);
                g.release();
            });

            // Get the next chunk of data available from the queue
            //let mut next_grant = consumer.read().unwrap();
            match consumer.read() {
                None => {
                    defmt::debug!("i2s_dma_handler :: no audio data available");
                    i2s_dma.lock(|dma| {
                        assert!(dma.is_some());
                        dma.as_mut().unwrap()
                                    .next_transfer(&ZEROES).unwrap()
                    });
                },
                Some(next_grant) => {
                    defmt::debug!("i2s_dma_handler :: next transfer {:#x} bytes", next_grant.len());

                    i2s_dma.lock(|dma| {
                        assert!(dma.is_some());
                        let b: &[u16] = cast_slice(unsafe { transmute::<&[u8], &'static [u8]>(&next_grant) });
                        dma.as_mut()
                           .unwrap()
                           .next_transfer(b).unwrap()
                    });
                    read_grant.replace(next_grant);
                }
            }

        }
    }

    #[task(binds = TIM2, local = [feedback_timer], shared = [feedback_clock_counter])]
    fn tim2(mut cx: tim2::Context) {
        defmt::debug!("TIM2 interrupt");
        let count = cx.local.feedback_timer.get_count();
        match count {
            None => defmt::info!("TIM2 interrupt but no TIR"),
            Some(i) => {
                cx.shared.feedback_clock_counter.lock(|counter| counter.add(i));
            },
        }
    }

    type I2sPeripheral = i2s::I2s<pac::SPI3, (
        PA15<Alternate<PushPull, 6>>,
        PC10<Alternate<PushPull, 6>>,
        PC7<Alternate<PushPull, 6>>,
        PC12<Alternate<PushPull, 6>>,
    )>;
    type I2sDevice = stm32_i2s_v12x::I2s<I2sPeripheral, TransmitMode<Data24Frame32>>;
    type I2sDmaTransfer = Transfer<Stream5<pac::DMA1>, I2sDevice, MemoryToPeripheral, &'static [u16], 0>;
}
