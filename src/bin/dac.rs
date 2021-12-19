#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {
    use as_slice::AsMutSlice;
    use stm32f4xx_hal::{
        i2s::{self, NoMasterClock},
        otg_fs::{USB, UsbBus, UsbBusType},
        pac::{self, TIM2},
        prelude::*,
        timer::{monotonic::MonoTimer, Timer},
        gpio::{Alternate, PushPull},
        gpio::gpioa::{PA4, PA5},
        gpio::gpioc::{PC10, PC12},
        dma::{
            config::{DmaConfig},
            MemoryToPeripheral,
            StreamsTuple,
            Transfer,
            Stream5,
            traits::StreamISR,
        },
    };

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use crabdac::uac::UsbAudioClass;
    use crabdac::timer::{UsbAudioFrequencyFeedback, CaptureChannel};

    use stm32_i2s_v12x::{self, MasterConfig, format::Data24Frame32, MasterClock, TransmitMode};

    unsafe impl<T: ReadTarget> ReadTarget for [T; 392] {}
    unsafe impl<T: WriteTarget> WriteTarget for [T; 392] {}

    type AUDIO_BUFFER = [u16; (96000 / 1000 + 2) * 2 * 2];

    // Shared resources go here
    #[shared]
    struct Shared {
        buffer: &'static mut AUDIO_BUFFER
    }

    // Local resources go here
    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_audio: UsbAudioClass<'static, UsbBusType>,
        i2s_dma: I2sDmaTransfer,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [ep_memory: [u32; 1024] = [0; 1024],
                    usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
                    usb_audio_stream_buf: [u8; 768] = [0; 768],
                    i2s_dma_buf: [u16; 1024] = [0; 1024],
                    buffer_a: AUDIO_BUFFER = [0; 392],
                    buffer_b: AUDIO_BUFFER = [0; 392],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(180.mhz())
            .pclk1(45.mhz())
            .pclk2(90.mhz())
            .i2s_apb1_clk(24571.khz())
            .require_pll48clk()
            .freeze();

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

        defmt::info!("init");

        let gpioa = cx.device.GPIOA.split();
        let gpioc = cx.device.GPIOC.split();

        let usb = USB {
            usb_global: cx.device.OTG_FS_GLOBAL,
            usb_device: cx.device.OTG_FS_DEVICE,
            usb_pwrclk: cx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        *cx.local.usb_bus = Some(UsbBus::new(usb, cx.local.ep_memory));

        let usb_audio = UsbAudioClass::new(cx.local.usb_bus.as_ref().unwrap(), cx.local.usb_audio_stream_buf);

        let usb_dev = UsbDeviceBuilder::new(cx.local.usb_bus.as_ref().unwrap(), UsbVidPid(0x1209, 0x0001))
            .manufacturer("Crabs Pty Ltd.")
            .product("CrabDAC")
            .serial_number("TEST")
            .device_class(0xff)
            .build();

        let i2s_dev = i2s::I2s::new(
            cx.device.SPI3,
            (
                gpioa.pa4.into_alternate(),
                gpioc.pc10.into_alternate(),
                i2s::NoMasterClock {},
                gpioc.pc12.into_alternate(),
            ),
            &clocks
        );
        let i2s_clock = i2s_dev.input_clock();

        let mut i2s = stm32_i2s_v12x::I2s::new(i2s_dev).configure_master_transmit(
                MasterConfig::with_sample_rate(
                    i2s_clock.0,
                    96000,
                    Data24Frame32,
                    stm32_i2s_v12x::format::FrameFormat::PhilipsI2s,
                    stm32_i2s_v12x::Polarity::IdleHigh,
                    MasterClock::Disable,
                ));
        i2s.set_dma_enabled(true);

        let dma1_streams = StreamsTuple::new(cx.device.DMA1);
        let dma_config = DmaConfig::default()
            .double_buffer(false)
            .fifo_enable(true)
            .fifo_threshold(stm32f4xx_hal::dma::config::FifoThreshold::HalfFull)
            .priority(stm32f4xx_hal::dma::config::Priority::VeryHigh)
            .transfer_complete_interrupt(true);
        let mut dma_transfer: I2sDmaTransfer =
            Transfer::init_memory_to_peripheral(dma1_streams.5, i2s, cx.local.buffer_b, None, dma_config);
        dma_transfer.start(|i2s| {
            defmt::info!("Started I2S DMA stream");
            i2s.enable();
        });

        // TODO wire up TIM2 for frequency feedback:
        // * External clock mode
        // * Trigger Source = ITR1
        // * Channel 1/2/3/4 = Input Capture direct mode
        //   * TIM2_CH2 on PA1 maybe?
        // * 0 prescaler
        // * Count-up
        // * Auto-reload = 2^32
        // * CKD = no clock division
        // * auto-reload preload = false?
        // * slave mode controller = ETR mode 1?
        // * ITR1 remap TIM2 ITR1 input -> USB OTG FS SOF
        //let feedback_timer = timer::Timer::new(cx.device.TIM2, &clocks);
        //let frequency_fb_timer = timer::Timer::new(cx.device.TIM2, &clocks);
        //let frequency_fb_timer_ch1 = gpioa.pa15.into_alternate::<1>();
        //let fftimer = UsbAudioFrequencyFeedback::<TIM2, PA5<Alternate<PushPull, 1>>, CaptureChannel<TIM2, 1>>::new(cx.device.TIM2, gpioa.pa5.into_alternate());
        let fftimer = UsbAudioFrequencyFeedback::<TIM2, PA5<Alternate<PushPull, 1>>, CaptureChannel<TIM2, 1>>::new(cx.device.TIM2, gpioa.pa5.into_alternate());

        task1::spawn().ok();

        // Setup the monotonic timer
        (
            Shared {
                buffer: cx.local.buffer_a
            },
            Local {
                // Initialization of local resources go here
                usb_dev,
                usb_audio,
                i2s_dma: dma_transfer,
            },
            init::Monotonics(mono),
        )
    }

    // Optional idle, can be removed if not needed.
    // Note that removing this will put the MCU to sleep when no task is running, and this
    // generally breaks RTT based printing.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    #[task]
    fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }

    #[task(binds = OTG_FS, priority = 3, local = [usb_dev, usb_audio])]
    fn otg_fs(cx: otg_fs::Context) {
        let otg_fs::LocalResources { usb_dev, usb_audio } = cx.local;
        while usb_dev.poll(&mut [usb_audio]) {
        };
    }

    #[task(binds = DMA1_STREAM5, priority = 3, local = [i2s_dma])]
    fn dma1_stream5(cx: dma1_stream5::Context) {
        defmt::trace!("DMA1_STREAM5 interrupt");
        let dma1_stream5::LocalResources { i2s_dma } = cx.local;
        if Stream5::<pac::DMA1>::get_transfer_complete_flag() {
            i2s_dma.clear_transfer_complete_interrupt();
        }
    }

    type I2sDmaTransfer = Transfer<
        Stream5<pac::DMA1>,
        stm32_i2s_v12x::I2s<
            i2s::I2s<pac::SPI3,
                (
                    PA4<Alternate<PushPull, 6>>,
                    PC10<Alternate<PushPull, 6>>,
                    NoMasterClock,
                    PC12<Alternate<PushPull, 6>>,
                ),
            >,
            TransmitMode<Data24Frame32>,
        >,
        MemoryToPeripheral,
        &'static mut [u16; 96000 / 1000 * 2 * 2],
        0,
    >;

    struct AudioBuffer<B> {
        buf: B
    }

    impl<B, Word> AudioBuffer<B>
    where
        B: DerefMut,
        B::Target: AsMutSlice<Element = Word>
    {
        fn new(buf: B) {
            Self { buf }
        }
    }
}
