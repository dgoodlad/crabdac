#![no_main]
#![no_std]

use crabdac as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {
    use stm32f4xx_hal::{
        Interrupt,
        pac,
        prelude::*,
        timer::{monotonic::MonoTimer, Timer},
    };

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
    type MicrosecMono = MonoTimer<Interrupt::TIM2, 1_000_000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
        let mono = Timer::new(cx.device.TIM2, &clocks).monotonic();

        defmt::info!("init");

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
