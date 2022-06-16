#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {

    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        (
            Shared {

            },
            Local {

            },
            init::Monotonics(

            ),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}
