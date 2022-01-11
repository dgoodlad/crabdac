use core::{marker::PhantomData, ops::Deref};

use stm32f4xx_hal::{
    dma::{
        traits::{DMASet, PeriAddress},
        Stream1,
        Stream3,
        Stream4,
        Stream5,
        Stream6,
        Stream7,
        MemoryToPeripheral,
        PeripheralToMemory,
    },
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
}

macro_rules! sai {
    ($SAI:ident) => {
        impl Instance for $SAI {
            fn ptr() -> *const sai1::RegisterBlock {
                <$SAI>::ptr() as *const _
            }
        }
    };
}

sai! { SAI1 }
sai! { SAI2 }

pub trait InstanceBlock {
    #[doc(hidden)]
    fn ptr() -> *const sai1::CH;
}

impl<SAI, const B: char> InstanceBlock for BlockX<SAI, B>
where
    SAI: Instance
{
    fn ptr() -> *const sai1::CH {
        match B {
            'A' => unsafe { &(*SAI::ptr()).cha as *const _ },
            'B' => unsafe { &(*SAI::ptr()).chb as *const _ },
            _ => panic!("Invalid SAI Block {:?}", B),
        }
    }
}

pub struct DisabledBlock<BLOCK>(PhantomData<BLOCK>);

impl<SAI, const B: char> DisabledBlock<BlockX<SAI, B>>
where
    SAI: Instance,
{
    pub fn slave_receiver<PINS>(pins: PINS) -> SaiBlock<BlockX<SAI, B>, Slave, Rx, PINS>
    where
        PINS: Pins<BlockX<SAI, B>, Slave>
    {
        match B {
            'A' => {  },
            'B' => {  },
            _ => { panic!("Invalid SAI Block {:?}", B) }
        }
        SaiBlock::new(pins)
    }
}

pub struct Sai<SAI> {
    _sai: SAI,
    block_a: Option<DisabledBlock<BlockA<SAI>>>,
    block_b: Option<DisabledBlock<BlockB<SAI>>>
}

impl<SAI> Sai<SAI>
where
    SAI: Instance,
{
    pub fn new(sai: SAI, _clocks: &Clocks) -> Sai<SAI> {
        Sai {
            _sai: sai,
            block_a: Some(DisabledBlock(PhantomData)),
            block_b: Some(DisabledBlock(PhantomData)),
        }
    }

    pub fn block_a(&mut self) -> Option<DisabledBlock<BlockA<SAI>>> {
        self.block_a.take()
    }

    pub fn block_b(&mut self) -> Option<DisabledBlock<BlockB<SAI>>> {
        self.block_b.take()
    }
}

pub struct Mclk;
pub struct Sck;
pub struct Fs;
pub struct Sd;

pub trait Pins<BLOCK, MODE> {}

impl<BLOCK, SCK, FS, SD> Pins<BLOCK, Slave>
    for (SCK, FS, SD)
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

pub struct NoPins;
impl<BLOCK, MODE> Pins<BLOCK, MODE> for NoPins {}

pub type NoMclk = NoPin;

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
pub type BlockA<SAI> = BlockX<SAI, 'A'>;
pub type BlockB<SAI> = BlockX<SAI, 'B'>;

pub struct Master;
pub struct Slave;

pub struct Rx;
pub struct Tx;

pub struct SaiBlock<BLOCK, MODE, DIRECTION, PINS> {
    _pins: PINS,
    _block: PhantomData<BLOCK>,
    _mode: PhantomData<MODE>,
    _direction: PhantomData<DIRECTION>,
}

impl<BLOCK, MODE, DIRECTION, PINS> SaiBlock<BLOCK, MODE, DIRECTION, PINS>
where
    PINS: Pins<BLOCK, MODE>
{
    pub fn new(pins: PINS) -> Self {
        SaiBlock { _pins: pins, _block: PhantomData, _mode: PhantomData, _direction: PhantomData }
    }
}

macro_rules! dma_map {
    ($(($Stream:ty, $C: literal, $Block:ty)),+ $(,)*) => {
        $(
            unsafe impl<MODE, PINS: Pins<$Block, MODE>>
                DMASet<$Stream, MemoryToPeripheral, $C> for
                SaiBlock<$Block, MODE, Tx, PINS> {}
            unsafe impl<MODE, PINS: Pins<$Block, MODE>>
                DMASet<$Stream, PeripheralToMemory, $C> for
                SaiBlock<$Block, MODE, Rx, PINS> {}
        )+
    };
}

// See RM0390 page 207, Table 29. DMA2 request mapping
dma_map!(
    // SAI1_A
    (Stream1<DMA2>, 0, BlockA<SAI1>),
    (Stream3<DMA2>, 0, BlockA<SAI1>),

    // SAI1_B
    (Stream5<DMA2>, 0, BlockB<SAI1>),
    (Stream4<DMA2>, 1, BlockB<SAI1>),

    // SAI2_A
    (Stream4<DMA2>, 3, BlockA<SAI2>),

    // SAI2_B
    (Stream7<DMA2>, 0, BlockB<SAI2>),
    (Stream6<DMA2>, 3, BlockB<SAI2>),
);

unsafe impl<SAI, MODE, DIRECTION, PINS> PeriAddress for SaiBlock<BlockA<SAI>, MODE, DIRECTION, PINS>
where
    SAI: Instance,
{
    #[inline(always)]
    fn address(&self) -> u32 {
        unsafe { &(*SAI::ptr()).cha.dr as *const _ as u32 }
    }

    type MemSize = u32;
}

unsafe impl<SAI, MODE, DIRECTION, PINS> PeriAddress for SaiBlock<BlockB<SAI>, MODE, DIRECTION, PINS>
where
    SAI: Instance,
{
    #[inline(always)]
    fn address(&self) -> u32 {
        unsafe { &(*SAI::ptr()).chb.dr as *const _ as u32 }
    }

    type MemSize = u32;
}
