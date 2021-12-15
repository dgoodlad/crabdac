#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {
    use stm32f4xx_hal::{
        i2s::{self, NoMasterClock},
        otg_fs::{USB, UsbBus},
        pac::{self, TIM2},
        prelude::*,
        timer::{monotonic::MonoTimer, Timer},
        gpio::{Alternate, PushPull},
        gpio::gpioa::{PA4, PA15},
        gpio::gpioc::{PC10, PC12},
        dma::{
            config::DmaConfig,
            MemoryToPeripheral,
            StreamsTuple,
            Transfer,
            Stream5,
        },
        timer,
    };

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use crabdac::uac::UsbAudioClass;

    use stm32_i2s_v12x::{self, MasterConfig, format::Data24Frame32, MasterClock, TransmitMode};

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_audio: UsbAudioClass<'static, UsbBus<USB>>,
        //i2s_dev: i2s::I2s<pac::SPI3, (PA4<Alternate<PushPull, 6>>,
        //                              PC10<Alternate<PushPull, 6>>,
        //                              NoMasterClock,
        //                              PC12<Alternate<PushPull, 6>>)>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [ep_memory: [u32; 1024] = [0; 1024],
                    usb_bus: Option<UsbBusAllocator<UsbBus<USB>>> = None,
                    usb_audio_stream_buf: [u8; 768] = [0; 768],
                    i2s_dma_buf: [u16; 1024] = [0; 1024],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(25.mhz())
            .sysclk(168.mhz())
            .pclk1(42.mhz())
            .pclk2(84.mhz())
            .i2s_clk(24576.khz())
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
            .memory_increment(true)
            .transfer_complete_interrupt(true);
        let mut dma_transfer: I2sDmaTransfer =
            Transfer::init_memory_to_peripheral(dma1_streams.5, i2s, cx.local.i2s_dma_buf, None, dma_config);

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
        let frequency_fb_timer = timer::Timer::new(cx.device.TIM2, &clocks);
        let frequency_fb_timer_ch1 = gpioa.pa15.into_alternate::<1>();

        task1::spawn().ok();

        // Setup the monotonic timer
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                usb_dev,
                usb_audio,
                //i2s_dev,
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
        &'static mut [u16; 1024],
        0,
    >;

    pub struct InputCaptureTimer<TIM, CH1> {
        pub(crate) tim: TIM,
        // TODO fix this generic
        pub(crate) ch1: CH1
    }

    impl InputCaptureTimer<pac::TIM2, PA15<Alternate<PushPull, 1>>> {
        pub fn new(tim: TIM2, ch1: PA15<Alternate<PushPull, 1>>) -> Self {
            Self {
                tim,
                ch1
            }
        }

        pub fn start(&self) {
            self.tim.cr2.write(|w| unsafe {
                // Map TIMx_CH1 pin to TI1 input
                w.ti1s().clear_bit()
            });
            self.tim.ccmr1_input().write(|w| unsafe { w
                .cc2s().bits(0b01)
                .ic2f().bits(0b0000)
            });
            self.tim.ccer.write(|w| unsafe { w
                .cc2p().clear_bit()
                .cc2np().clear_bit()
            });
            self.tim.smcr.write(|w| unsafe { w
                .sms().bits(0b111)
                .ts().bits(0b110)
            });
            self.tim.or.write(|w| unsafe { w
                // Internally map the USB OTG FS SOF signal to the timer's ITR1
                // trigger input
                .itr1_rmp().bits(0b10)
            });
        }
    }
}
