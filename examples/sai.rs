#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {

    use hal::dma::MemoryToPeripheral;
    use hal::dma::Stream4;
    use hal::dma::StreamsTuple;
    use hal::dma::Transfer;
    use hal::dma::config::DmaConfig;
    use hal::gpio::Alternate;
    use hal::gpio::PushPull;
    use hal::gpio::gpioa::PA9;
    use hal::gpio::gpiob::PB12;
    use hal::gpio::gpiob::PB9;
    use hal::gpio::gpioc::PC0;
    use hal::pac::DMA2;
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
    type SaiTx = sai::Transmitter<sai::BlockB<pac::SAI1>, SaiPins, sai::Master>;
    type SaiDmaTransfer = Transfer<Stream4<DMA2>, SaiTx, MemoryToPeripheral, &'static mut [u32; 2], 1>;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        dma: SaiDmaTransfer,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [audio_data: [u32; 2] = [0x000000ff, 0x00ffffff]])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("INIT :: Configuring Clocks");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
            .hclk(168.mhz())
            .pclk1(42.mhz())
            .pclk2(84.mhz())
            .require_pll48clk()
            .sai1_clk(49152.khz())
            .freeze();

        assert!(clocks.is_pll48clk_valid());

        defmt::info!("INIT :: Clocks configured");
        defmt::info!("INIT :: SAI Clock: {:?}", defmt::Debug2Format(&clocks.sai1_clk()));

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
        let mut tx = block.master_transmitter(pins);
        tx.configure();

        let dma2 = cx.device.DMA2;
        let streams = StreamsTuple::new(dma2);
        let dma_config = DmaConfig::default()
            .memory_increment(false)
            .transfer_complete_interrupt(true);
        let mut dma_transfer = Transfer::init_memory_to_peripheral(streams.4, tx, cx.local.audio_data, None, dma_config);
        dma_transfer.start(|sai| sai.enable());

        defmt::info!("INIT :: Finished");

        (
            Shared {},
            Local { dma: dma_transfer },
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

    #[task(binds = DMA2_STREAM4, priority = 3, local = [dma])]
    fn sai_dma(cx: sai_dma::Context) {
        let dma: &mut SaiDmaTransfer = cx.local.dma;

        dma.clear_transfer_complete_interrupt();
        unsafe {
            dma.next_transfer_with(|buffer, _| {
                (buffer, ())
            }).unwrap();
        }
    }
}
