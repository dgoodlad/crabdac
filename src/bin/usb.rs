#![no_main]
#![no_std]

use crabdac as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {
    use stm32f4xx_hal::{
        otg_fs::{USB, UsbBus},
        pac,
        prelude::*,
        timer::{monotonic::MonoTimer, Timer},
    };

    use usb_device::{
        prelude::*,
        class_prelude::{UsbBusAllocator}, class::UsbClass
    };

    use crabdac::uac::UsbAudioClass;

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
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM2, 1_000_000>;

    #[init(local = [ep_memory: [u32; 4096] = [0; 4096],
                    usb_bus: Option<UsbBusAllocator<UsbBus<USB>>> = None,
                    usb_audio_stream_buf: [u8; 768] = [0; 768],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .require_pll48clk()
            .freeze();

        let mono = Timer::new(cx.device.TIM2, &clocks).monotonic();

        defmt::info!("init");

        let gpioa = cx.device.GPIOA.split();

        let usb = USB {
            usb_global: cx.device.OTG_FS_GLOBAL,
            usb_device: cx.device.OTG_FS_DEVICE,
            usb_pwrclk: cx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        *cx.local.usb_bus = Some(UsbBus::new(usb, cx.local.ep_memory));
        let usb_bus = cx.local.usb_bus.as_ref().unwrap();

        let usb_audio = UsbAudioClass::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x0001))
            .manufacturer("Crabs Pty Ltd.")
            .product("CrabDAC")
            .serial_number("TEST")
            .device_class(0xff)
            .build();

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
        defmt::debug!("OTG_FS Interrupt");
        let otg_fs::LocalResources { usb_dev, usb_audio } = cx.local;
        while usb_dev.poll(&mut [usb_audio]) {
            defmt::debug!("Polling usb audio");
        };
    }
}
