use core::{marker::PhantomData, ops::Deref};

pub mod dma;

use hal::time::Hertz;

use crate::hal::{
    self,
    gpio::{
        gpioa,
        gpiob,
        gpioc,
        gpiod,
        gpioe,
        gpiof,
        gpiog,
    },
    pac::{
        sai1,
        DMA2,
        SAI1,
        SAI2,
    },
    rcc::Clocks, gpio::{Alternate, PushPull, NoPin}
};

// Implemented by all SAI instances
pub trait Instance:
    Deref<Target = sai1::RegisterBlock>
{
    #[doc(hidden)]
    fn ptr() -> *const sai1::RegisterBlock;

    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Option<Hertz>;
}

macro_rules! sai {
    ($SAI:ident, $clock:ident) => {
        impl Instance for $SAI {
            fn ptr() -> *const sai1::RegisterBlock {
                <$SAI>::ptr() as *const _
            }

            fn clock(clocks: &Clocks) -> Option<Hertz> {
                clocks.$clock()
            }
        }
    };
}

sai! { SAI1, sai1_clk }
sai! { SAI2, sai2_clk }

pub struct Sai<SAI> {
    sai: SAI,
    clock: Hertz,
}

impl<SAI> Sai<SAI>
where
    SAI: Instance,
{
    pub fn new(sai: SAI, clocks: &Clocks) -> Sai<SAI> {
        // Ensures that the SAI peripheral's clock is configured
        let clock = SAI::clock(clocks).unwrap();

        Sai { sai, clock }
    }

    pub fn clock(&self) -> Hertz {
        self.clock
    }

    // TODO add functions to configure external sync

    fn free(self) -> SAI {
        self.sai
    }
}

pub struct Mclk;
pub struct Sck;
pub struct Fs;
pub struct Sd;
pub struct NoMasterClock;

pub trait Pins<BLOCK, MODE> {}

impl<BLOCK, SCK, FS, SD> Pins<BLOCK, Slave>
    for (SCK, FS, SD, NoMasterClock)
where
    SCK: SaiPin<BLOCK, Sck>,
    FS: SaiPin<BLOCK, Fs>,
    SD: SaiPin<BLOCK, Sd>,
{}

impl<BLOCK, SCK, FS, SD, MCLK> Pins<BLOCK, Master>
    for (SCK, FS, SD, MCLK)
where
    SCK: SaiPin<BLOCK, Sck>,
    FS: SaiPin<BLOCK, Fs>,
    SD: SaiPin<BLOCK, Sd>,
    MCLK: SaiPin<BLOCK, Mclk>,
{}

pub trait SaiPin<Pin, Block> {}

macro_rules! sai_pin {
    ( $(<$SAI:ty, $Block:expr, $Pin:ty> for [$($gpio:ident::$PX:ident<$A:literal>),*]),*) => {
        $(
            $(
                impl SaiPin<$Pin, BlockX<$SAI, $Block>> for $gpio::$PX<Alternate<PushPull, $A>> {}
            )*
        )*
    };
}

sai_pin! {
    <SAI1, 'A', Fs> for [gpioa::PA3<6>, gpioe::PE4<6>],
    <SAI1, 'A', Sck> for [gpiob::PB10<6>, gpioe::PE5<6>],
    <SAI1, 'A', Sd> for [gpiob::PB2<6>, gpioc::PC1<6>, gpiod::PD6<6>, gpioe::PE6<6>],
    <SAI1, 'A', Mclk> for [gpioe::PE2<6>],
    <SAI1, 'B', Fs> for [gpiob::PB9<6>, gpiof::PF9<6>],
    <SAI1, 'B', Sck> for [gpiob::PB12<6>, gpiof::PF8<6>],
    <SAI1, 'B', Sd> for [gpioa::PA9<6>, gpioe::PE3<6>, gpiof::PF6<6>],
    <SAI1, 'B', Mclk> for [gpioc::PC0<6>, gpiof::PF7<6>],

    <SAI2, 'A', Fs> for [gpiod::PD12<10>],
    <SAI2, 'A', Sck> for [gpiod::PD13<10>],
    <SAI2, 'A', Sd> for [gpiod::PD11<10>],
    <SAI2, 'A', Mclk> for [gpioe::PE0<10>],
    <SAI2, 'B', Fs> for [gpioe::PE13<10>, gpiog::PG9<10>],
    <SAI2, 'B', Sck> for [gpioe::PE12<10>],
    <SAI2, 'B', Sd> for [gpioe::PE11<10>, gpiof::PF11<10>, gpiog::PG10<10>],
    <SAI2, 'B', Mclk> for [gpioa::PA1<10>, gpioe::PE14<10>]
}

pub struct BlockX<SAI, const B: char>(PhantomData<SAI>);

impl<SAI: Instance, const B: char> BlockX<SAI, B> {
    fn new() -> Self {
        Self(PhantomData)
    }

    #[inline(always)]
    fn ch(&mut self) -> &'static sai1::CH {
        match B {
            'A' => unsafe { &(*SAI::ptr()).cha },
            'B' => unsafe { &(*SAI::ptr()).chb },
            _ => panic!("Invalid SAI Block {:?}", B),
        }

    }

    pub fn master_receiver<PINS: Pins<BlockX<SAI, B>, Master>>(self, pins: PINS) -> Receiver<BlockX<SAI, B>, PINS, Master, Disabled> {
        Receiver::master(self, pins)
    }

    pub fn slave_receiver<PINS: Pins<BlockX<SAI, B>, Slave>>(self, pins: PINS) -> Receiver<BlockX<SAI, B>, PINS, Slave, Disabled> {
        Receiver::slave(self, pins)
    }
}

pub type BlockA<SAI> = BlockX<SAI, 'A'>;
pub type BlockB<SAI> = BlockX<SAI, 'B'>;

pub struct Blocks<SAI> {
    pub sai: Sai<SAI>,
    pub a: BlockA<SAI>,
    pub b: BlockB<SAI>,
}

impl<SAI: Instance> Blocks<SAI> {
    /// Splits the SAI peripheral into its two sub-blocks
    pub fn new(sai: SAI, clocks: &Clocks) -> Self {
        Self {
            sai: Sai::new(sai, clocks),
            a: BlockX::new(),
            b: BlockX::new(),
        }
    }

    pub fn release(sai: Sai<SAI>, _a: BlockA<SAI>, _b: BlockB<SAI>) -> SAI {
        sai.free()
    }
}

pub struct Master;
pub struct Slave;

pub struct Rx;
pub struct Tx;

pub struct Enabled;
pub struct Disabled;

pub struct Receiver<BLOCK, PINS, MODE, ENABLED> {
    block: BLOCK,
    pins: PINS,
    _mode: PhantomData<MODE>,
    _enabled: PhantomData<ENABLED>,
}

impl<BLOCK, PINS, MODE, ENABLED> Receiver<BLOCK, PINS, MODE, ENABLED> {
    #[inline(always)]
    pub fn release(self) -> (BLOCK, PINS) {
        (self.block, self.pins)
    }
}

impl<SAI: Instance, PINS, MODE, const B: char> Receiver<BlockX<SAI, B>, PINS, MODE, Disabled>
where
    PINS: Pins<BlockX<SAI, B>, MODE>
{
    #[inline(always)]
    fn master(mut block: BlockX<SAI, B>, pins: PINS) -> Self {
        block.ch().cr1.modify(|_, w| w.mode().master_rx());
        Receiver {
            block,
            pins,
            _mode: PhantomData,
            _enabled: PhantomData,
        }
    }

    #[inline(always)]
    fn slave(mut block: BlockX<SAI, B>, pins: PINS) -> Self {
        block.ch().cr1.modify(|_, w| w.mode().slave_rx());
        Receiver {
            block,
            pins,
            _mode: PhantomData,
            _enabled: PhantomData
        }
    }

    #[inline(always)]
    pub fn set_protocol(&mut self) {
        self.block.ch().cr1.modify(|_, w| w.prtcfg().free());
    }
}

pub struct Transmitter<BLOCK, PINS, MODE, ENABLED> {
    block: BLOCK,
    pins: PINS,
    _mode: PhantomData<MODE>,
    _enabled: PhantomData<ENABLED>,
}

impl <BLOCK, PINS, MODE> Transmitter<BLOCK, PINS, MODE, Disabled> {
    #[inline(always)]
    pub fn release(self) -> (BLOCK, PINS) {
        (self.block, self.pins)
    }
}

impl<SAI: Instance, PINS, MODE, const B: char> Transmitter<BlockX<SAI, B>, PINS, MODE, Disabled>
where
    PINS: Pins<BlockX<SAI, B>, MODE>
{
    #[inline(always)]
    fn master(mut block: BlockX<SAI, B>, pins: PINS) -> Transmitter<BlockX<SAI, B>, PINS, Master, Disabled> {
        block.ch().cr1.modify(|_, w| w.mode().master_tx());
        Transmitter {
            block,
            pins,
            _mode: PhantomData,
            _enabled: PhantomData,
        }
    }

    #[inline(always)]
    fn slave(mut block: BlockX<SAI, B>, pins: PINS) -> Transmitter<BlockX<SAI, B>, PINS, Slave, Disabled> {
        block.ch().cr1.modify(|_, w| w.mode().master_tx());
        Transmitter {
            block,
            pins,
            _mode: PhantomData,
            _enabled: PhantomData,
        }
    }

    #[inline(always)]
    pub fn enable(mut self) -> Transmitter<BlockX<SAI, B>, PINS, Slave, Enabled> {
        self.block.ch().cr1.modify(|_, w| w.saien().enabled());
        Transmitter {
            block: self.block,
            pins: self.pins,
            _mode: PhantomData,
            _enabled: PhantomData,
        }
    }
}

impl<SAI: Instance, PINS, MODE, const B: char> Transmitter<BlockX<SAI, B>, PINS, MODE, Enabled>
{
    pub fn write(&mut self, data: u32) {
        self.block.ch().dr.write(|w| unsafe { w.data().bits(data) });
    }
}
