#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI3])]
mod app {
    use core::intrinsics::transmute;

    use bbqueue;
    use bbqueue::GrantR;
    use embedded_dma::ReadBuffer;
    use embedded_dma::StaticReadBuffer;
    use stable_deref_trait::StableDeref;
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::pac;
    use stm32f4xx_hal::{
        timer::{monotonic::MonoTimer, Timer},
    };

    const CHANNELS: u32 = 2;
    const SAMPLE_RATE: u32 = 96000;
    const USB_FRAME_RATE: u32 = 1000;
    const SAMPLE_WORD_SIZE: usize = 4;
    const MAX_FRAME_SIZE: usize = ((SAMPLE_RATE / USB_FRAME_RATE + 1) * CHANNELS) as usize * SAMPLE_WORD_SIZE;
    const BUFFER_SIZE: usize = MAX_FRAME_SIZE * 2;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        producer: bbqueue::Producer<'static, BUFFER_SIZE>,
        consumer: bbqueue::Consumer<'static, BUFFER_SIZE>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [audio_queue: bbqueue::BBBuffer<BUFFER_SIZE> = bbqueue::BBBuffer::new()])]
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

    #[task(local = [producer])]
    fn usb_handler(cx: usb_handler::Context) {
        let usb_handler::LocalResources { producer } = cx.local;
        let mut grant = producer.grant_exact(MAX_FRAME_SIZE).unwrap();

        let len = MAX_FRAME_SIZE - (SAMPLE_WORD_SIZE * CHANNELS as usize);
        let buf: &mut [u8] = grant.buf();
        // TODO fill the buffer from the USB Endpoint memory
        grant.commit(len);
    }

    #[task(local = [consumer])]
    fn i2s_dma_handler(cx: i2s_dma_handler::Context) {
        let i2s_dma_handler::LocalResources { consumer } = cx.local;
        let grant = consumer.read().unwrap();
        let len = grant.len();
        let buf = DmaReadBuffer(grant);
        unsafe { buf.static_read_buffer(); }
    }

    struct DmaReadBuffer<T>(T);

    impl<'a, const N: usize> core::ops::Deref for DmaReadBuffer<GrantR<'a, N>> {
        type Target = [u16];

        fn deref(&self) -> &[u16] {
            unsafe { transmute::<&[u8], &[u16]>(self.0.deref()) }
        }
    }

    unsafe impl<'a, const N: usize> StableDeref for DmaReadBuffer<GrantR<'a, N>> {}
}
