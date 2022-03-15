#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {
    use stm32f4xx_hal as hal;
    use hal::prelude::*;
    use hal::pac;
    use stm32f4xx_hal::timer::MonoTimerUs;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<pac::TIM5>;

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("INIT :: Configuring Clocks");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(25.MHz())
            .sysclk(96.MHz())
            .hclk(96.MHz())
            .pclk1(48.MHz())
            .pclk2(96.MHz())
            //.i2s_apb1_clk(98400.kHz())
            .require_pll48clk()
            .freeze();

        assert!(clocks.is_pll48clk_valid());

        let mono = cx.device.TIM5.monotonic_us(&clocks);

        defmt::info!("INIT :: Finished");

        (
            Shared {},
            Local {},
            init::Monotonics(mono),
        )
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("IDLE");

        loop {
            continue;
        }
    }
}
