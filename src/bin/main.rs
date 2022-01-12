#![no_main]
#![no_std]

use crabdac as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI2])]
mod app {
    use crabdac::{hal, uac::simple_stereo_output::SimpleStereoOutput};
    use hal::{
        prelude::*,
        pac,

        dma::{
            config::DmaConfig,
            traits::StreamISR,
            MemoryToPeripheral,
            Stream4,
            StreamsTuple,
            Transfer,
        },

        gpio::{
            gpioa::{self, *},
            gpiob::{self, *},
            gpioc::{self, *},
            Alternate,
            PushPull,
        },

        otg_hs::{
            USB,
            UsbBus,
            UsbBusType,
        },

        timer::{
            Timer,
            monotonic::MonoTimer,
        }
    };

    use crabdac::sai;

    use bbqueue;
    use bytemuck::cast_slice;

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    type SaiPins = (
        PB12<Alternate<PushPull, 6>>,
        PB9<Alternate<PushPull, 6>>,
        PA9<Alternate<PushPull, 6>>,
        PC0<Alternate<PushPull, 6>>,
    );
    type SaiTx = sai::Transmitter<sai::BlockB<pac::SAI1>, SaiPins, sai::Master>;
    type SaiDmaTransfer = Transfer<Stream4<pac::DMA2>, SaiTx, MemoryToPeripheral, &'static [u32], 1>;

    #[shared]
    struct Shared {
        #[lock_free]
        sai_dma_transfer: SaiDmaTransfer,
    }

    #[local]
    struct Local {
        audio_data_producer: bbqueue::Producer<'static, BUFFER_SIZE>,
        audio_data_consumer: bbqueue::Consumer<'static, BUFFER_SIZE>,

        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_audio: SimpleStereoOutput<'static, UsbBusType>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimer<pac::TIM5, 1_000_000>;

    const CHANNELS: u32 = 2;
    const SAMPLE_RATE: u32 = 96000;
    const SLOT_SIZE: u32 = 32;
    const USB_FRAME_RATE: u32 = 1000;
    const USB_AUDIO_FRAME_SIZE: usize = ((SAMPLE_RATE / USB_FRAME_RATE) + 1 * CHANNELS * SLOT_SIZE / 8) as usize;
    const BUFFER_SIZE: usize = USB_AUDIO_FRAME_SIZE * 2;
    const SAI_DMA_SIZE: usize = 256 as usize;

    #[init(local = [
        mute_buffer: [u32; SAI_DMA_SIZE] = [0; SAI_DMA_SIZE],
        audio_data_buffer: bbqueue::BBBuffer<BUFFER_SIZE> = bbqueue::BBBuffer::new(),
        usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
        usb_ep_memory: [u32; 1024] = [0; 1024],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("INIT :: Configuring Clocks");

        let rcc: hal::rcc::Rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
            .hclk(168.mhz())
            .pclk1(42.mhz())
            .pclk2(84.mhz())
            .sai1_clk(49152.khz())
            .require_pll48clk()
            .freeze();

        assert!(clocks.is_pll48clk_valid());
        //assert!((clocks.sai1_clk().unwrap().0 as f64 - 49152.khz().0 as f64) / (49152.khz().0 as f64) < 0.0002);

        defmt::info!("INIT :: Clocks configured.");
        defmt::info!("INIT ::   HCLK          : {}", clocks.hclk().0);
        defmt::info!("INIT ::   PCLK1         : {}", clocks.pclk1().0);
        defmt::info!("INIT ::   PCLK2         : {}", clocks.pclk2().0);
        defmt::info!("INIT ::   USB (pll48clk): {}", clocks.pll48clk().map(|h| h.0));
        defmt::info!("INIT ::   SAI (sai1_clk): {}", clocks.sai1_clk().map(|h| h.0));

        let mono = Timer::new(cx.device.TIM5, &clocks).monotonic();

        defmt::info!("INIT :: Allocating audio data buffer");
        let (audio_data_producer, audio_data_consumer) = cx.local.audio_data_buffer.try_split().unwrap();

        let gpioa: gpioa::Parts = cx.device.GPIOA.split();
        let gpiob: gpiob::Parts = cx.device.GPIOB.split();
        let gpioc: gpioc::Parts = cx.device.GPIOC.split();


        defmt::info!("INIT :: Configuring SAI");
        let sai_pins: SaiPins = (
            gpiob.pb12.into_alternate(),
            gpiob.pb9.into_alternate(),
            gpioa.pa9.into_alternate(),
            gpioc.pc0.into_alternate(),
        );

        let sai_transmitter: SaiTx = sai::Blocks::new(cx.device.SAI1, &clocks)
            .b.master_transmitter(sai_pins);

        let mute_buffer: &'static [u32] = cx.local.mute_buffer;
        let mut sai_dma_transfer = Transfer::init_memory_to_peripheral(
            StreamsTuple::new(cx.device.DMA2).4,
            sai_transmitter,
            mute_buffer,
            None,
            DmaConfig::default()
                .double_buffer(false)
                .memory_increment(true)
                .transfer_complete_interrupt(true),
        );
        sai_dma_transfer.start(|tx| tx.enable());


        defmt::info!("INIT :: Configuring USB");
        let usb = USB {
            usb_global: cx.device.OTG_HS_GLOBAL,
            usb_device: cx.device.OTG_HS_DEVICE,
            usb_pwrclk: cx.device.OTG_HS_PWRCLK,
            pin_dm: gpiob.pb14.into_alternate(),
            pin_dp: gpiob.pb15.into_alternate(),
            hclk: clocks.hclk(),
        };

        cortex_m::asm::delay(clocks.hclk().0);

        *cx.local.usb_bus = Some(UsbBus::new(usb, cx.local.usb_ep_memory));
        let usb_audio = SimpleStereoOutput::new(
            cx.local.usb_bus.as_ref().unwrap(),
            SAMPLE_RATE,
            SLOT_SIZE as usize / 8,
            24,
        );
        let usb_dev = UsbDeviceBuilder::new(cx.local.usb_bus.as_ref().unwrap(), UsbVidPid(0x1209, 0x0001))
            .manufacturer("Crabs Pty Ltd.")
            .product("CrabDAC")
            .serial_number("420.69")
            .composite_with_iads()
            .max_packet_size_0(8)
            .self_powered(true)
            .build();

        defmt::info!("INIT :: Finished");

        (
            Shared {
                sai_dma_transfer,
            },
            Local {
                audio_data_producer,
                audio_data_consumer,
                usb_dev,
                usb_audio,
            },
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

    #[task(binds = DMA2_STREAM4,
           local = [audio_data_consumer],
           shared = [sai_dma_transfer])]
    fn sai_dma_handler(cx: sai_dma_handler::Context) {
        let consumer: &mut bbqueue::Consumer<'static, BUFFER_SIZE> = cx.local.audio_data_consumer;
        let transfer = cx.shared.sai_dma_transfer;

        if Stream4::<pac::DMA2>::get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();

            consumer.read().map(|grant| {
                let bytes: &'static [u8] = if grant.len() > 8 {
                    unsafe { &grant.as_static_buf()[0..8] }
                } else {
                    unsafe { &grant.as_static_buf() }
                };
                let words: &'static [u32] = cast_slice(bytes);

                unsafe {
                    transfer.next_transfer_with(|_, _| {
                        (words, ())
                    }).unwrap();
                }
                grant.release(8);
            }).unwrap();

        }
    }

    #[task(binds = OTG_HS, local = [audio_data_producer, usb_dev, usb_audio])]
    fn usb_handler(cx: usb_handler::Context) {
        let producer: &mut bbqueue::Producer<'static, BUFFER_SIZE> = cx.local.audio_data_producer;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.local.usb_dev;
        let usb_audio: &mut SimpleStereoOutput<UsbBusType> = cx.local.usb_audio;

        while usb_dev.poll(&mut [usb_audio]) {
            if usb_audio.audio_data_available {
                producer.grant_exact(USB_AUDIO_FRAME_SIZE).and_then(|mut grant| {
                    let bytes_received = usb_audio.read_audio_data(&mut grant).unwrap();
                    grant.commit(bytes_received);
                    Ok(())
                }).unwrap();
            }
        }
    }
}
