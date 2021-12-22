#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {

    use hal::timer::Timer;
    use stm32f4xx_hal as hal;
    use hal::prelude::*;
    use hal::pac;
    use stm32f4xx_hal::timer::monotonic::MonoTimer;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("INIT :: Configuring Clocks");

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

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

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
