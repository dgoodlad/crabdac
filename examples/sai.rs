#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {

    use hal::gpio::Alternate;
    use hal::gpio::PushPull;
    use hal::gpio::gpioa::PA9;
    use hal::gpio::gpiob::PB12;
    use hal::gpio::gpiob::PB9;
    use hal::gpio::gpioc::PC0;
    use hal::timer::Timer;
    use stm32f4xx_hal as hal;
    use hal::prelude::*;
    use hal::pac;
    use stm32f4xx_hal::timer::monotonic::MonoTimer;

    use crabdac::sai;

    type SaiPins = (
        PB12<Alternate<PushPull, 6>>,
        PB9<Alternate<PushPull, 6>>,
        PA9<Alternate<PushPull, 6>>,
        PC0<Alternate<PushPull, 6>>,
    );
    //type SaiTx = sai::Transmitter<sai::BlockB<pac::SAI1>, SaiPins, sai::Master, sai::Enabled>;

    enum SaiTx {
        Enabled(sai::Transmitter<sai::BlockB<pac::SAI1>, SaiPins, sai::Master, sai::Enabled>),
        Disabled(sai::Transmitter<sai::BlockB<pac::SAI1>, SaiPins, sai::Master, sai::Disabled>),
    }

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        sai: SaiTx,
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
            .require_pll48clk()
            .sai1_clk(49142857)
            .freeze();

        assert!(clocks.is_pll48clk_valid());

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();

        let sai1: pac::SAI1 = cx.device.SAI1;
        let blocks = sai::Blocks::new(sai1, &clocks);
        let block = blocks.b;
        let sck = gpiob.pb12;
        let fs = gpiob.pb9;
        let sd = gpioa.pa9;
        let mclk = gpioc.pc0;
        let pins: SaiPins = (
            sck.into_alternate(),
            fs.into_alternate(),
            sd.into_alternate(),
            mclk.into_alternate()
        );
        let tx = block.master_transmitter(pins);

        defmt::info!("INIT :: Finished");

        (
            Shared {},
            Local { sai: SaiTx::Disabled(tx) },
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
