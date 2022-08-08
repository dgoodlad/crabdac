#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use stm32f4xx_hal::{
        prelude::*,
        timer::MonoTimerUs,
        timer::Timer,
        pac::TIM5
    };

    use ringbuf::*;

    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {

    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<TIM5>;

    const TYPICAL_SAMPLES_PER_FRAME: u32 = 96;
    const BUFFER_SIZE: usize = 256;


    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
                        .use_hse(25.MHz())
                        .sysclk(96.MHz())
                        .hclk(96.MHz())
                        .pclk1(48.MHz())
                        .pclk2(96.MHz())
                        .freeze();

        let mono = cx.device.TIM5.monotonic_us(&clocks);

        let mut ringbuf = StaticRb::<u32, BUFFER_SIZE>::default();
        let (mut p, mut c) = ringbuf.split_ref();


        for i in 0..(TYPICAL_SAMPLES_PER_FRAME * 10000) {
            if i % TYPICAL_SAMPLES_PER_FRAME == 0 {
                for s in i..i+TYPICAL_SAMPLES_PER_FRAME { p.push(s).unwrap(); }
            }

            if i % (BUFFER_SIZE as u32 / 2) == 0 {
                let (s1, s2) = c.as_slices();
                defmt::println!("s1: {}, s2: {}", s1, s2);
                defmt::println!("*s1: {:x}", s1.as_ptr());
                defmt::println!("*s2: {:x}", s2.as_ptr());
                unsafe { c.advance(BUFFER_SIZE / 2); }
            }
        }

        (
            Shared {

            },
            Local {

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
