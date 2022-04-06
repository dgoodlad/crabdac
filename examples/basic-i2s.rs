//!
//! # I2S example for STM32F411
//!
//! This application demonstrates I2S communication with the DAC on an STM32F411E-DISCO board.
//! Unlike `i2s-audio-out`, this example uses DMA instead of writing one sample at a time.
//!
//! # Hardware required
//!
//! * STM32F407G-DISC1 or STM32F411E-DISCO evaluation board
//! * Headphones or speakers with a headphone plug
//!
//! # Procedure
//!
//! 1. Connect the headphones or speakers to the headphone jack on the evaluation board
//!    (warning: the DAC may produce a powerful signal that becomes a very loud sound.
//!    Set the speaker volume to minimum, or do not put on the headphones.)
//! 2. Load this compiled application on the microcontroller and run it
//!
//! Expected behavior: the speakers/headphones emit a continuous 750 Hz tone
//!
//! # Pins and addresses
//!
//! * PD4 -> DAC ~RESET (pulled low)
//!
//! * PB9 -> SDA (pulled high)
//! * PB6 -> SCL (pulled high)
//!
//! * PC7 -> MCLK
//! * PC10 -> SCK (bit clock)
//! * PC12 -> SD
//! * PA4 -> WS
//!
//! DAC I2C address 0x94
//!

#![no_std]
#![no_main]

use crabdac as _;

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use stm32_i2s_v12x::format::{Data16Frame16, FrameFormat, Data24Frame32};
use stm32_i2s_v12x::{MasterClock, MasterConfig, Polarity, TransmitMode};

use stm32f4xx_hal::dma::config::DmaConfig;
use stm32f4xx_hal::dma::MemoryToPeripheral;
use stm32f4xx_hal::dma::{Stream4, StreamsTuple, Transfer};
use stm32f4xx_hal::gpio::{Alternate, PB12, PB13, PB15, PA3};
use stm32f4xx_hal::i2s::I2s;
use stm32f4xx_hal::pac::{interrupt, Interrupt};
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals};
use stm32f4xx_hal::pac::{DMA1, SPI2};
use stm32f4xx_hal::prelude::*;

const SINE_SAMPLES: usize = 64;

/// A sine wave spanning 64 samples
///
/// With a sample rate of 48 kHz, this produces a 750 Hz tone.
//const SINE_750: [i32; SINE_SAMPLES] = [
//    0, 3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169, 25329, 27244, 28897, 30272, 31356,
//    32137, 32609, 32767, 32609, 32137, 31356, 30272, 28897, 27244, 25329, 23169, 20787, 18204,
//    15446, 12539, 9511, 6392, 3211, 0, -3211, -6392, -9511, -12539, -15446, -18204, -20787, -23169,
//    -25329, -27244, -28897, -30272, -31356, -32137, -32609, -32767, -32609, -32137, -31356, -30272,
//    -28897, -27244, -25329, -23169, -20787, -18204, -15446, -12539, -9511, -6392, -3211,
//];
const SINE_750: [i32; SINE_SAMPLES] = [
    8388608,9210835,10025144,10823692,11598789,12342970,13049069,13710285,
    14320250,14873090,15363481,15786700,16138671,16416005,16616031,16736823,
    16777216,16736823,16616031,16416005,16138671,15786700,15363481,14873090,
    14320250,13710285,13049069,12342970,11598789,10823692,10025144,9210835,
    8388608,7566381,6752072,5953524,5178427,4434246,3728147,3066931,
    2456966,1904126,1413735,990516,638545,361211,161185,40393,
    0,40393,161185,361211,638545,990516,1413735,1904126,
    2456966,3066931,3728147,4434246,5178427,5953524,6752072,7566381,
];

#[entry]
fn main() -> ! {
    // Make a copy of SINE_750 with three differences:
    // 1. It's a way to get a &'static mut
    // 2. Its type is u16 instead of i16
    // 3. Each sample is repeated (for the left and right channels)
    static mut SINE_750_MUT: [u16; SINE_SAMPLES * 2 * 2] = [0; SINE_SAMPLES * 2 * 2];

    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    // The 86 MHz frequency can be divided to get a sample rate very close to 48 kHz.
    let clocks = rcc.cfgr.use_hse(25.MHz()).i2s_clk(98304.kHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let mut _delay = cp.SYST.delay(&clocks);

    // I2S pins: (WS, CK, MCLK, SD) for I2S2
    let i2s_pins = (
        gpiob.pb12.into_alternate(),
        gpiob.pb13.into_alternate(),
        gpioa.pa3.into_alternate(),
        gpiob.pb15.into_alternate(),
    );
    // let hal_i2s = I2s::new(dp.SPI3, i2s_pins, &clocks);
    // or
    let hal_i2s = dp.SPI2.i2s(i2s_pins, &clocks);
    let i2s_clock = hal_i2s.input_clock();

    // Audio timing configuration:
    // Sample rate 96 kHz
    // 32 bits per frame * 2 channels -> SCK rate 6.144 MHz
    // MCK frequency = 256 * sample rate -> MCK rate 24.576 MHz (also equal to 4 * SCK rate)
    let sample_rate = 96000;

    let i2s = stm32_i2s_v12x::I2s::new(hal_i2s);
    let mut i2s = i2s.configure_master_transmit(MasterConfig::with_sample_rate(
        i2s_clock.raw(),
        sample_rate,
        Data24Frame32,
        FrameFormat::PhilipsI2s,
        Polarity::IdleHigh,
        MasterClock::Enable,
    ));
    i2s.set_dma_enabled(true);

    for (dma_words, sample) in SINE_750_MUT.chunks_mut(4).zip(SINE_750.iter()) {
        dma_words[0] = (sample & 0xffff) as u16;
        dma_words[1] = (sample >> 16) as u16;
        dma_words[2] = (sample & 0xffff) as u16;
        dma_words[3] = (sample >> 16) as u16;
    }

    // Set up DMA: DMA 1 stream 4 channel 0 memory -> peripheral
    let dma1_streams = StreamsTuple::new(dp.DMA1);
    let dma_config = DmaConfig::default()
        .memory_increment(true)
        .transfer_complete_interrupt(true);
    let mut dma_transfer: I2sDmaTransfer =
        Transfer::init_memory_to_peripheral(dma1_streams.4, i2s, SINE_750_MUT, None, dma_config);

    dma_transfer.start(|i2s| i2s.enable());
    // Hand off transfer to interrupt handler
    cortex_m::interrupt::free(|cs| *G_TRANSFER.borrow(cs).borrow_mut() = Some(dma_transfer));
    // Enable interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_STREAM4);
    }

    loop {
        // Don't WFI. That can cause problems attaching the debugger.
    }
}

type I2sDmaTransfer = Transfer<
    Stream4<DMA1>,
    stm32_i2s_v12x::I2s<
        I2s<
            SPI2,
            (
                PB12<Alternate<5>>,
                PB13<Alternate<5>>,
                PA3<Alternate<5>>,
                PB15<Alternate<5>>,
            ),
        >,
        TransmitMode<Data24Frame32>,
    >,
    MemoryToPeripheral,
    &'static mut [u16; SINE_SAMPLES * 2 * 2],
    0,
>;

/// DMA transfer handoff from main() to interrupt handler
static G_TRANSFER: Mutex<RefCell<Option<I2sDmaTransfer>>> = Mutex::new(RefCell::new(None));

/// This interrupt handler runs when DMA 1 finishes a transfer to the I2S peripheral
#[interrupt]
fn DMA1_STREAM4() {
    static mut TRANSFER: Option<I2sDmaTransfer> = None;

    let transfer = unsafe { TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TRANSFER.borrow(cs).replace(None).unwrap())
    })};

    transfer.clear_transfer_complete_interrupt();
    unsafe {
        transfer
            .next_transfer_with(|buffer, _active_buffer| {
                // Transfer again with the same buffer
                (buffer, ())
            })
            .unwrap();
    }
}
