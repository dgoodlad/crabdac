#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use stm32f4xx_hal::{
        prelude::*,
        timer::MonoTimerUs,
        pac::{TIM5, TIM2}, qei::Qei, gpio::{PA0, PA1, Output, PushPull, Alternate, PC13},
        hal::Direction as RotaryDirection,
    };

    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        rotary_encoder: Qei<TIM2, (PA0<Alternate<1>>, PA1<Alternate<1>>)>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<TIM5>;

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

        let gpioa = cx.device.GPIOA.split();
        let gpioc = cx.device.GPIOC.split();

        let led = gpioc.pc13.into_push_pull_output();

        let rotary_encoder_pins = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
        let rotary_encoder_timer = cx.device.TIM2;
        let rotary_encoder = Qei::new(rotary_encoder_timer, rotary_encoder_pins);

        volume::spawn().unwrap();

        (
            Shared {

            },
            Local {
                led,
                rotary_encoder,

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

    #[task(local = [led, rotary_encoder, count: u32 = 0])]
    fn volume(cx: volume::Context) {
        let volume::LocalResources { count, led, rotary_encoder } = cx.local;

        volume::spawn_after(10.millis()).unwrap();

        let new_count = rotary_encoder.count();
        if new_count != *count {
            let diff = new_count.wrapping_sub(*count) as i16;

            defmt::info!("{} - {} = {}", new_count, *count, diff);

            match rotary_encoder.direction() {
                RotaryDirection::Upcounting => led.set_low(),
                RotaryDirection::Downcounting => led.set_high(),
            }

            *count = new_count;
        }
    }
}
