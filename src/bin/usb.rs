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

    use usb_device::prelude::*;

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM2, 1_000_000>;

    #[init]
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
            pin_dm: gpioa.pa11.into_alternate::<10>(),
            pin_dp: gpioa.pa12.into_alternate::<10>(),
            hclk: clocks.hclk(),
        };

        let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
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

    // TODO: Add tasks
    #[task]
    fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }
}
