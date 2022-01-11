use core::marker::PhantomData;

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
        PeripheralToMemory, Transfer,
    },
    pac::{DMA2, SAI1, SAI2}};

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
