#![no_main]
#![no_std]

use crabdac as _;
use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use crabdac::{
        decibels,
        hal,
        uac::{simple_stereo_output::SimpleStereoOutput, ClockCounter},
        timer::{UsbAudioFrequencyFeedback, CaptureChannel}
    };
    use crate::ALLOCATOR;

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
            Alternate, NoPin,
        },

        otg_fs::{
            USB,
            UsbBus,
            UsbBusType,
        },

        timer::MonoTimerUs, i2s::NoMasterClock,
    };

    use aligned::{Aligned, A4};

    use bbqueue::{self, GrantR};
    use bytemuck::cast_slice;

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use stm32_i2s_v12x::{self, TransmitMode, MasterConfig, format::{Data24Frame32, FrameFormat}, Polarity, MasterClock};

    type Sample = fixed::FixedI32<fixed::types::extra::U31>;

    type I2sPins = (
        // WS
        PB12<Alternate<5>>,
        // CK
        PB13<Alternate<5>>,
        // MCLK
        PA3<Alternate<5>>,
        // SD
        PB15<Alternate<5>>,
    );

    type I2sTx = stm32_i2s_v12x::I2s<hal::i2s::I2s<pac::SPI2, I2sPins>, TransmitMode<Data24Frame32>>;

    type I2sDmaTransfer = Transfer<
        Stream4<pac::DMA1>,
        I2sTx,
        MemoryToPeripheral,
        &'static [u16],
        0
    >;

    #[shared]
    struct Shared {
        #[lock_free]
        i2s_dma_transfer: I2sDmaTransfer,

        feedback_clock_counter: u32,
    }

    #[local]
    struct Local {
        audio_data_producer: bbqueue::Producer<'static, BUFFER_SIZE>,
        audio_data_consumer: bbqueue::Consumer<'static, BUFFER_SIZE>,
        audio_data_read_grant: Option<GrantR<'static, BUFFER_SIZE>>,

        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_audio: SimpleStereoOutput<'static, UsbBusType>,

        audio_feedback_timer: UsbAudioFrequencyFeedback<pac::TIM2, PA5<Alternate<1>>>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<pac::TIM5>;

    const CHANNELS: u32 = 2;
    const SAMPLE_RATE: u32 = 96000;
    const SLOT_SIZE: u32 = 32;
    const USB_FRAME_RATE: u32 = 1000;
    const USB_AUDIO_FRAME_SIZE: usize = (((SAMPLE_RATE / USB_FRAME_RATE) + 1) * CHANNELS * SLOT_SIZE / 8) as usize;
    const BUFFER_SIZE: usize = USB_AUDIO_FRAME_SIZE * 4;
    const I2S_DMA_SIZE: usize = 64 as usize;

    #[init(local = [
        heap: [u8; 1024] = [0; 1024],
        mute_buffer: [u16; I2S_DMA_SIZE] = [0; I2S_DMA_SIZE],
        audio_data_buffer: bbqueue::BBBuffer<BUFFER_SIZE> = bbqueue::BBBuffer::new(),
        usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
        usb_ep_memory: [u32; 320] = [0; 320],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("INIT :: Configuring Clocks");

        let rcc_peripheral: hal::pac::RCC = cx.device.RCC;
        let rcc: hal::rcc::Rcc = rcc_peripheral.constrain();
        let clocks = rcc.cfgr
            .use_hse(25.MHz())
            .sysclk(96.MHz())
            .hclk(96.MHz())
            .pclk1(48.MHz())
            .pclk2(96.MHz())
            .i2s_clk(98304.kHz())
            .require_pll48clk()
            .freeze();

        assert!(clocks.is_pll48clk_valid());

        defmt::info!("INIT :: Clocks configured.");
        defmt::info!("INIT ::   HCLK          : {}", clocks.hclk().raw());
        defmt::info!("INIT ::   PCLK1         : {}", clocks.pclk1().raw());
        defmt::info!("INIT ::   PCLK2         : {}", clocks.pclk2().raw());
        defmt::info!("INIT ::   USB (pll48clk): {}", clocks.pll48clk().map(|h| h.raw()));
        defmt::info!("INIT ::   I2S           : {}", clocks.i2s_clk().map(|h| h.raw()));

        defmt::info!("INIT :: Configuring monotonic timer");
        let mono = cx.device.TIM5.monotonic_us(&clocks);

        defmt::info!("INIT :: Configuring global allocator");
        unsafe { ALLOCATOR.init(cx.local.heap.as_ptr() as usize, 1024) }

        defmt::info!("INIT :: Allocating audio data buffer");
        let (audio_data_producer, audio_data_consumer) = cx.local.audio_data_buffer.try_split().unwrap();

        let gpioa: gpioa::Parts = cx.device.GPIOA.split();
        let gpiob: gpiob::Parts = cx.device.GPIOB.split();
        let gpioc: gpioc::Parts = cx.device.GPIOC.split();

        defmt::info!("INIT :: Configuring I2S");
        let i2s_pins: I2sPins = (
            gpiob.pb12.into_alternate(), // WS
            gpiob.pb13.into_alternate(), // CK
            gpioa.pa3.into_alternate(), // MCLK
            gpiob.pb15.into_alternate(), // SD
        );

        let hal_i2s = cx.device.SPI2.i2s(i2s_pins, &clocks);
        let i2s_config = MasterConfig::with_sample_rate(
            clocks.i2s_clk().unwrap().raw(),
            96000,
            Data24Frame32,
            FrameFormat::PhilipsI2s,
            Polarity::IdleHigh,
            MasterClock::Enable
        );
        defmt::info!("i2s config: {:?}", defmt::Debug2Format(&i2s_config));
        let mut i2s_transmitter: I2sTx = stm32_i2s_v12x::I2s::new(hal_i2s)
            .configure_master_transmit(i2s_config);
        i2s_transmitter.set_dma_enabled(true);

        let mute_buffer: &'static [u16] = cx.local.mute_buffer;
        let mut i2s_dma_transfer = Transfer::init_memory_to_peripheral(
            StreamsTuple::new(cx.device.DMA1).4,
            i2s_transmitter,
            mute_buffer,
            None,
            DmaConfig::default()
                .priority(hal::dma::config::Priority::High)
                .double_buffer(false)
                .memory_increment(true)
                .transfer_complete_interrupt(true),
        );
        i2s_dma_transfer.start(|tx| tx.enable());

        defmt::info!("INIT :: Configuring USB");
        let usb = USB {
            usb_global: cx.device.OTG_FS_GLOBAL,
            usb_device: cx.device.OTG_FS_DEVICE,
            usb_pwrclk: cx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

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

        let audio_feedback_timer: UsbAudioFrequencyFeedback<pac::TIM2, PA5<Alternate<1>>> =
            UsbAudioFrequencyFeedback::new(cx.device.TIM2, CaptureChannel::Channel1, gpioa.pa5.into_alternate());
        audio_feedback_timer.start();

        defmt::info!("INIT :: Finished");

        (
            Shared {
                i2s_dma_transfer,
                feedback_clock_counter: (256 * SAMPLE_RATE * 4 / USB_FRAME_RATE) << 4, // number of mclk pulses (256 * F_s) in 4 usb frames
            },
            Local {
                audio_data_producer,
                audio_data_consumer,
                audio_data_read_grant: None,
                usb_dev,
                usb_audio,
                audio_feedback_timer,
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

    #[task(binds = DMA1_STREAM4,
           priority = 3,
           local = [audio_data_consumer, audio_data_read_grant],
           shared = [i2s_dma_transfer])]
    fn i2s_dma_handler(cx: i2s_dma_handler::Context) {
        let consumer: &mut bbqueue::Consumer<'static, BUFFER_SIZE> = cx.local.audio_data_consumer;
        let old_grant: &mut Option<bbqueue::GrantR<'static, BUFFER_SIZE>> = cx.local.audio_data_read_grant;
        let transfer: &mut I2sDmaTransfer = cx.shared.i2s_dma_transfer;

        if Stream4::<pac::DMA1>::get_transfer_complete_flag() {
            match old_grant.take() {
                Some(g) => {
                    defmt::debug!("I2S :: Dropping a grant of {} bytes", g.len());
                    drop(g);
                },
                None => {}
            }

            match consumer.read() {
                Ok(mut grant) => {
                    defmt::debug!("I2S :: Processing a grant of {} bytes", grant.len());
                    let bytes: &'static [u8] = if grant.len() > I2S_DMA_SIZE {
                        grant.to_release(I2S_DMA_SIZE);
                        unsafe { &grant.as_static_buf()[0..I2S_DMA_SIZE] }
                    } else {
                        grant.to_release(grant.len());
                        unsafe { &grant.as_static_buf() }
                    };
                    old_grant.replace(grant);
                    let words: &'static [u16] = cast_slice(bytes);
                    transfer.next_transfer(words).unwrap();
                },
                Err(_) => {
                    unsafe { transfer.next_transfer_with(|buf, _| (buf, ())).unwrap(); }
                }
            }

        }
    }

    #[task(binds = OTG_FS,
           priority = 2,
           local = [
               audio_data_producer,
               usb_dev,
               usb_audio,
               usb_audio_buf: Aligned<A4, [u8; USB_AUDIO_FRAME_SIZE]> = Aligned([0; USB_AUDIO_FRAME_SIZE])
           ],
           shared = [feedback_clock_counter]
    )]
    fn usb_handler(cx: usb_handler::Context) {
        let producer: &mut bbqueue::Producer<'static, BUFFER_SIZE> = cx.local.audio_data_producer;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.local.usb_dev;
        let usb_audio: &mut SimpleStereoOutput<UsbBusType> = cx.local.usb_audio;
        let usb_audio_buf = &mut **cx.local.usb_audio_buf;
        let mut feedback_clock_counter = cx.shared.feedback_clock_counter;

        while usb_dev.poll(&mut [usb_audio]) {
            if usb_audio.audio_feedback_needed {
                match usb_audio.write_raw_feedback(feedback_clock_counter.lock(|i| *i)) {
                    Ok(_) => defmt::debug!("USB :: Feedback OK"),
                    Err(_) => defmt::warn!("USB :: Feedback ERR"),
                }
            }

            if usb_audio.audio_data_available {
                let bytes_received = usb_audio.read_audio_data(usb_audio_buf).unwrap();
                defmt::debug!("USB :: received {} bytes of audio data", bytes_received);

                let volume_db = usb_audio.volume;
                let volume_amp = match volume_db {
                    0 => Sample::from_num(1 as i32),
                    _ => Sample::from_bits(decibels::DB[((volume_db >> 8) + 127) as usize]),
                };

                let samples = cast_slice::<u8, i32>(&usb_audio_buf[0..bytes_received]).iter()
                    .cloned()
                    .map(|sample| Sample::from_bits(sample))
                    .map(|sample| sample * volume_amp)
                    .map(|sample| sample.to_le_bytes());

                match producer.grant_exact(bytes_received) {
                    Ok(mut grant) => {
                        for (sample_bytes, grant_bytes) in samples.zip(grant.chunks_exact_mut(4)) {
                            grant_bytes[0] = sample_bytes[2];
                            grant_bytes[1] = sample_bytes[3];
                            grant_bytes[2] = sample_bytes[0];
                            grant_bytes[3] = sample_bytes[1];
                        }
                        grant.commit(bytes_received);
                    },
                    Err(e) => defmt::warn!("USB :: grant failed {:?}", defmt::Debug2Format(&e)),
                }
            }
        }
    }

    #[task(binds = TIM2, priority = 1, local = [audio_feedback_timer], shared = [feedback_clock_counter])]
    fn tim2(mut cx: tim2::Context) {
        static mut COUNTER: ClockCounter = ClockCounter{ticks: 0, frames: 0, mck_to_fs_ratio: 8};

        let count = cx.local.audio_feedback_timer.get_count();
        match count {
            None => defmt::warn!("TIM2 interrupt but no count"),
            Some(i) => {
                unsafe {
                    COUNTER.add(i);
                    if COUNTER.frames >= 4 {
                        cx.shared.feedback_clock_counter.lock(|counter| *counter = COUNTER.ticks << 4);
                        COUNTER.clear();
                    }
                }
            }
        }
    }
}
