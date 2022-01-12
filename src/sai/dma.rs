use crate::hal;

use hal::{
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
};

use super::*;

macro_rules! dma_map {
    ($(($Stream:ty, $C: literal, $Block:ty)),+ $(,)*) => {
        $(
            unsafe impl<MODE, PINS: Pins<$Block, MODE>> DMASet<$Stream, MemoryToPeripheral, $C> for Transmitter<$Block, PINS, MODE> {}
            unsafe impl<MODE, PINS: Pins<$Block, MODE>> DMASet<$Stream, PeripheralToMemory, $C> for Receiver<$Block, PINS, MODE> {}
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

unsafe impl<SAI, MODE, PINS> PeriAddress for Transmitter<BlockA<SAI>, PINS, MODE>
where
    SAI: Instance,
{
    #[inline(always)]
    fn address(&self) -> u32 {
        unsafe { &(*SAI::ptr()).cha.dr as *const _ as u32 }
    }

    type MemSize = u32;
}

unsafe impl<SAI, MODE, PINS> PeriAddress for Transmitter<BlockB<SAI>, PINS, MODE>
where
    SAI: Instance,
{
    #[inline(always)]
    fn address(&self) -> u32 {
        unsafe { &(*SAI::ptr()).chb.dr as *const _ as u32 }
    }

    type MemSize = u32;
}
