#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {
    use bbqueue;
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::pac;
    use stm32f4xx_hal::{
        timer::{monotonic::MonoTimer, Timer},
    };

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        producer: bbqueue::Producer<'static, 768>,
        consumer: bbqueue::Consumer<'static, 768>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [audio_queue: bbqueue::BBBuffer<768> = bbqueue::BBBuffer::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init: clocks");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(180.mhz())
            .pclk1(45.mhz())
            .pclk2(90.mhz())
            .i2s_apb1_clk(24571.khz())
            .require_pll48clk()
            .freeze();

        defmt::info!("init: monotonic timer");

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

        let (producer, consumer) = cx.local.audio_queue.try_split().unwrap();

        (
            Shared {},
            Local {
                producer,
                consumer,
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
}
