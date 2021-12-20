#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {
    use core::intrinsics::transmute;

    use bbqueue;
    use bbqueue::GrantR;
    use bytemuck::*;
    use embedded_dma::StaticReadBuffer;
    use stable_deref_trait::StableDeref;
    use stm32_i2s_v12x::MasterConfig;
    use stm32_i2s_v12x::Polarity;
    use stm32_i2s_v12x::TransmitMode;
    use stm32_i2s_v12x::format::Data24Frame32;
    use stm32_i2s_v12x::format::FrameFormat;
    use stm32f4xx_hal::dma::traits::StreamISR;
    use stm32f4xx_hal::dma::MemoryToPeripheral;
    use stm32f4xx_hal::dma::Stream5;
    use stm32f4xx_hal::dma::StreamsTuple;
    use stm32f4xx_hal::dma::Transfer;
    use stm32f4xx_hal::dma::config::DmaConfig;
    use stm32f4xx_hal::gpio::Alternate;
    use stm32f4xx_hal::gpio::PushPull;
    use stm32f4xx_hal::gpio::gpioa::PA4;
    use stm32f4xx_hal::gpio::gpioc::PC7;
    use stm32f4xx_hal::gpio::gpioc::PC10;
    use stm32f4xx_hal::gpio::gpioc::PC12;
    use stm32f4xx_hal::i2s;
    use stm32f4xx_hal::i2s::NoMasterClock;
    use stm32f4xx_hal::{prelude::*, pac, timer::{monotonic::MonoTimer, Timer, monotonic::fugit, monotonic::fugit::ExtU32}};

    const CHANNELS: u32 = 2;
    //const SAMPLE_RATE: u32 = 96000;
    const SAMPLE_RATE: u32 = 96000;
    const USB_FRAME_RATE: u32 = 1000;
    const SAMPLE_WORD_SIZE: usize = 4;
    const MAX_FRAME_SIZE: usize = ((SAMPLE_RATE / USB_FRAME_RATE + 1) * CHANNELS) as usize * SAMPLE_WORD_SIZE;
    const FRAME_HEADER_SIZE: usize = 2;
    const BUFFER_SIZE: usize = (MAX_FRAME_SIZE + FRAME_HEADER_SIZE) * 3;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        producer: bbqueue::framed::FrameProducer<'static, BUFFER_SIZE>,
        consumer: bbqueue::framed::FrameConsumer<'static, BUFFER_SIZE>,
        write_grant: Option<bbqueue::framed::FrameGrantW<'static, BUFFER_SIZE>>,
        read_grant: Option<bbqueue::framed::FrameGrantR<'static, BUFFER_SIZE>>,
        i2s_dma: I2sDmaTransfer,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    #[init(local = [audio_queue: bbqueue::BBBuffer<BUFFER_SIZE> = bbqueue::BBBuffer::new(),
                    zeroes: [u16; 768] = [0; 768],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init: clocks");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(180.mhz())
            .pclk1(45.mhz())
            .pclk2(90.mhz())
            .i2s_apb1_clk(98400.khz())
            .require_pll48clk()
            .freeze();

        defmt::info!("init: monotonic timer");

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

        let (producer, consumer) = cx.local.audio_queue.try_split_framed().unwrap();

        let gpioa = cx.device.GPIOA.split();
        let gpioc = cx.device.GPIOC.split();

        let i2s_periph = I2sPeripheral::new(
            cx.device.SPI3, (
                gpioa.pa4.into_alternate(),
                gpioc.pc10.into_alternate(),
                gpioc.pc7.into_alternate(),
                gpioc.pc12.into_alternate(),
            ),
            &clocks
        );

        let i2s_clock = i2s_periph.input_clock();
        defmt::info!("I2S Clock: {}Hz", i2s_clock.0);

        let i2s_config = MasterConfig::with_sample_rate(
            i2s_clock.0,
            SAMPLE_RATE,
            Data24Frame32,
            FrameFormat::PhilipsI2s,
            Polarity::IdleHigh,
            stm32_i2s_v12x::MasterClock::Enable
        );

        let mut i2s: I2sDevice = stm32_i2s_v12x::I2s::new(i2s_periph)
            .configure_master_transmit(i2s_config);
        i2s.set_dma_enabled(true);

        let dma1_streams = StreamsTuple::new(cx.device.DMA1);
        let dma_config = DmaConfig::default()
            .double_buffer(false)
            .fifo_enable(true)
            .fifo_threshold(stm32f4xx_hal::dma::config::FifoThreshold::HalfFull)
            .priority(stm32f4xx_hal::dma::config::Priority::VeryHigh)
            .transfer_complete_interrupt(true);
        let mut i2s_dma: I2sDmaTransfer =
            Transfer::init_memory_to_peripheral(dma1_streams.5, i2s, cx.local.zeroes, None, dma_config);

        i2s_dma.start(|i2s| {
            defmt::info!("Started I2S DMA stream");
            i2s.enable();
            //usb_handler::spawn_after(1.millis()).ok();
            usb_handler::spawn().ok();
        });

        //i2s_dma_handler::spawn().ok();

        (
            Shared {},
            Local {
                producer,
                consumer,
                i2s_dma,
                read_grant: None,
                write_grant: None,
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
        let mut sample_1: [u8; 4] = [0x00, 0x02, 0x04, 0x06];
        let mut sample_2: [u8; 4] = [0x01, 0x03, 0x05, 0x07];

        usb_handler::spawn_after(1.millis()).unwrap();

        defmt::debug!("usb_handler");
        defmt::debug!("usb_handler :: requesting frame up to {:#x} bytes", MAX_FRAME_SIZE);

        let usb_handler::LocalResources { producer } = cx.local;
        let mut grant = match producer.grant(MAX_FRAME_SIZE) {
            Ok(grant) => grant,
            Err(_) => { defmt::debug!("Dropped USB Frame"); return; }
        };

        let len = MAX_FRAME_SIZE; //MAX_FRAME_SIZE - (SAMPLE_WORD_SIZE * CHANNELS as usize);

        // TODO fill the buffer from the USB Endpoint memory
        for chunk in grant[0..len].chunks_mut(8) {
            chunk[0..4].copy_from_slice(&sample_1);
            chunk[4..8].copy_from_slice(&sample_2);

            sample_1[1] += 1;
            sample_2[1] += 1;
            sample_1[2] += 1;
            sample_2[2] += 1;
        }

        grant.commit(len);

        defmt::debug!("usb_handler :: committed {:#x} bytes", len);
    }

    #[task(binds = DMA1_STREAM5, priority = 3, local = [consumer, i2s_dma, read_grant])]
    fn i2s_dma_handler(cx: i2s_dma_handler::Context) {
        let i2s_dma_handler::Context { local } = cx;
        let i2s_dma_handler::LocalResources { consumer, i2s_dma, read_grant } = local;

        defmt::debug!("i2s_dma_handler");

        if Stream5::<pac::DMA1>::get_transfer_complete_flag() {
            defmt::debug!("i2s_dma_handler :: transfer complete");

            read_grant.take().map(|g| {
                let len = g.len();
                defmt::debug!("i2s dma handler :: released {:#x} bytes", len);
                g.release();
            });

            // Get the next chunk of data available from the queue
            let mut next_grant = consumer.read().unwrap();

            defmt::debug!("i2s_dma_handler :: next transfer {:#x} bytes", next_grant.len());
            i2s_dma.next_transfer(cast_slice(unsafe { transmute::<&[u8], &'static [u8]>(&next_grant) })).unwrap();
            read_grant.replace(next_grant);
        }
    }

    type I2sPeripheral = i2s::I2s<pac::SPI3, (
        PA4<Alternate<PushPull, 6>>,
        PC10<Alternate<PushPull, 6>>,
        PC7<Alternate<PushPull, 6>>,
        PC12<Alternate<PushPull, 6>>,
    )>;
    type I2sDevice = stm32_i2s_v12x::I2s<I2sPeripheral, TransmitMode<Data24Frame32>>;
    type I2sDmaTransfer = Transfer<Stream5<pac::DMA1>, I2sDevice, MemoryToPeripheral, &'static [u16], 0>;
}
