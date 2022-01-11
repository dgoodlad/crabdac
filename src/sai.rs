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
        RCC,
    },
    rcc::{self, Clocks}, gpio::{Alternate, PushPull, Pin}
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

pub struct Sai<SAI> {
    sai: SAI,
}

impl<SAI> Sai<SAI>
where
    SAI: Instance,
{
    pub fn new(sai: SAI, clocks: &Clocks) -> Sai<SAI> {
        Sai { sai }
    }

    // pub fn split(self) -> Blocks {
    //     unsafe {
    //         let rcc = &(*RCC::ptr());

    //         self.0
    //     }
    // }
}

pub struct Mclk;
pub struct Sck;
pub struct Fs;
pub struct Sd;

pub trait Pins<SAI, BLOCK> {}

impl<SAI, BLOCK, SCK, FS, SD> Pins<SAI, BLOCK>
    for (SCK, FS, SD)
where
    SCK: SaiPin<SAI, BLOCK, Sck>,
    FS: SaiPin<SAI, BLOCK, Fs>,
    SD: SaiPin<SAI, BLOCK, Sd>,
{}

impl<SAI, BLOCK, SCK, FS, SD, MCLK> Pins<SAI, BLOCK>
    for (SCK, FS, SD, MCLK)
where
    SCK: SaiPin<SAI, BLOCK, Sck>,
    FS: SaiPin<SAI, BLOCK, Fs>,
    SD: SaiPin<SAI, BLOCK, Sd>,
    MCLK: SaiPin<SAI, BLOCK, Mclk>,
{}

pub struct NoPins;
impl Pins<SAI1, A> for NoPins {}
impl Pins<SAI1, B> for NoPins {}
impl Pins<SAI2, A> for NoPins {}
impl Pins<SAI2, B> for NoPins {}

pub trait SaiPin<Pin, Block, SAI> {}

macro_rules! sai_pin {
    ( $(<$SAI:ty, $Block:ty, $Pin:ty> for [$($gpio:ident::$PX:ident<$A:literal>),*]),*) => {
        $(
            $(
                impl SaiPin<$Pin, $Block, $SAI> for $gpio::$PX<Alternate<PushPull, $A>> {}
            )*
        )*
    };
}

sai_pin! {
    <SAI1, A, Fs> for [gpioa::PA3<6>, gpioe::PE4<6>],
    <SAI1, A, Sck> for [gpiob::PB10<6>, gpioe::PE5<6>],
    <SAI1, A, Sd> for [gpiob::PB2<6>, gpioc::PC1<6>, gpiod::PD6<6>, gpioe::PE6<6>],
    <SAI1, A, Mclk> for [gpioe::PE2<6>],
    <SAI1, B, Fs> for [gpiob::PB9<6>, gpiof::PF9<6>],
    <SAI1, B, Sck> for [gpiob::PB12<6>, gpiof::PF8<6>],
    <SAI1, B, Sd> for [gpioa::PA9<6>, gpioe::PE3<6>, gpiof::PF6<6>],
    <SAI1, B, Mclk> for [gpioc::PC0<6>, gpiof::PF7<6>],

    <SAI2, A, Fs> for [gpiod::PD12<10>],
    <SAI2, A, Sck> for [gpiod::PD13<10>],
    <SAI2, A, Sd> for [gpiod::PD11<10>],
    <SAI2, A, Mclk> for [gpioe::PE0<10>],
    <SAI2, B, Fs> for [gpioe::PE13<10>, gpiog::PG9<10>],
    <SAI2, B, Sck> for [gpioe::PE12<10>],
    <SAI2, B, Sd> for [gpioe::PE11<10>, gpiof::PF11<10>, gpiog::PG10<10>],
    <SAI2, B, Mclk> for [gpioa::PA1<10>, gpioe::PE14<10>]
}

// macro_rules! pin {
//     ( $(<$Pin:ty, $SAI:ty, $BLOCK:ty> for [$($gpio:ident::$PX:ident<$A:literal>),*]),*) => {
//         $(
//             $(
//                 impl<MODE> $Pin<$SAI, $BLOCK> for $gpio::$PX<MODE> {
//                 }
//             )
//         )
//     };
// }

pub struct Blocks<SAI, PinsA, PinsB>
where
    PinsA: Pins<SAI, A>,
    PinsB: Pins<SAI, B>,
{
    sai: SAI,
    pins_a: PinsA,
    pins_b: PinsB,
}

pub struct A;
pub struct B;

pub mod mode {
    struct Master;
    struct Slave;
}

pub mod dir {
    pub struct Rx;
    pub struct Tx;
}

pub struct SaiBlock<PERI, BLOCK, MODE, DIRECTION>(
    PERI,
    PhantomData<BLOCK>,
    PhantomData<MODE>,
    PhantomData<DIRECTION>
);

impl<PERI, AB, MODE> SaiBlock<PERI, AB, MODE, dir::Tx> {
}

macro_rules! dma_map {
    ($(($Stream:ty, $C: literal, $SaiPeripheral:ty, $Block:ty)),+ $(,)*) => {
        $(
            unsafe impl<MODE> DMASet<$Stream, MemoryToPeripheral, $C> for SaiBlock<$SaiPeripheral, $Block, MODE, dir::Tx> {}
            unsafe impl<MODE> DMASet<$Stream, PeripheralToMemory, $C> for SaiBlock<$SaiPeripheral, $Block, MODE, dir::Rx> {}
        )+
    };
}

// See RM0390 page 207, Table 29. DMA2 request mapping
dma_map!(
    // SAI1_A
    (Stream1<DMA2>, 0, SAI1, A),
    (Stream3<DMA2>, 0, SAI1, A),

    // SAI1_B
    (Stream5<DMA2>, 0, SAI1, B),
    (Stream4<DMA2>, 1, SAI1, B),

    // SAI2_A
    (Stream4<DMA2>, 3, SAI2, A),

    // SAI2_B
    (Stream7<DMA2>, 0, SAI2, B),
    (Stream6<DMA2>, 3, SAI2, B),
);

unsafe impl<MODE, DIRECTION> PeriAddress for SaiBlock<SAI1, A, MODE, DIRECTION> {
    #[inline(always)]
    fn address(&self) -> u32 {
        &(self.0).cha.dr as *const _ as u32
    }

    type MemSize = u32;
}

unsafe impl<MODE, DIRECTION> PeriAddress for SaiBlock<SAI1, B, MODE, DIRECTION> {
    #[inline(always)]
    fn address(&self) -> u32 {
        &(self.0).chb.dr as *const _ as u32
    }

    type MemSize = u32;
}
