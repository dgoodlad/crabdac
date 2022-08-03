use core::ops::{Deref, DerefMut};

use stm32f4xx_hal::gpio::{self, Alternate};
use stm32f4xx_hal::pac::{self, TIM2};
use stm32f4xx_hal::pac::RCC;
use stm32f4xx_hal::rcc::{Enable, Reset, BusTimerClock};

pub trait Pins<TIM> {}

macro_rules! etr_impl {
    ( $( $TIM:ident, $PINX:ident, $AF:literal; )+ ) => {
        $(
            impl<Otype> Pins<pac::$TIM> for gpio::$PINX<Alternate<$AF, Otype>> {  }
        )+
    };
}

etr_impl!(
    TIM2, PA0, 1;
    TIM2, PA5, 1;
    TIM2, PA15, 1;

    TIM1, PA12, 1;
);

pub struct SofTimer<TIM, PINS>
where
    PINS: Pins<TIM>,
{
    tim: TIM,
    pins: PINS,
}

impl<TIM, PINS> Deref for SofTimer<TIM, PINS>
where
    PINS: Pins<TIM>,
{
    type Target = TIM;
    fn deref(&self) -> &Self::Target {
        &self.tim
    }
}

impl<TIM, PINS> DerefMut for SofTimer<TIM, PINS>
where
    PINS: Pins<TIM>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.tim
    }
}

impl<PINS> SofTimer<TIM2, PINS>
where
    PINS: Pins<TIM2>,
{
    pub fn new(tim: TIM2, pins: PINS) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        TIM2::enable(&rcc);
        TIM2::reset(&rcc);

        // Reset (disable) the counter prescaler
        tim.psc.reset();

        // Remap ITR1 to OTG_FS_SOF
        tim.or.modify(|_r, w| unsafe { w.itr1_rmp().bits(0b10) });

        // Select slave mode trigger to be ITR1 (mapped to OTG_FS_SOF above)
        tim.smcr.modify(|_r, w| w.ts().itr1());

        // Select External Clock Mode 2, clocking the timer by the input on the
        // given ETR pin
        tim.smcr.modify(|_r, w| w.ece().enabled());

        // Configure the slave mode controller in reset mode
        tim.smcr.modify(|_r, w| w.sms().reset_mode());

        // Configure channel 1 in input capture mode mapped to TRC (ITR1 = OTR_FS_SOF)
        tim.ccmr1_input().modify(|_r, w| w.cc1s().trc());

        // Enable capture channel 1
        tim.ccer.modify(|_r, w| w.cc1e().set_bit());

        // Enable interrupts for capture channel 1
        tim.dier.modify(|_r, w| w.cc1ie().set_bit());

        // Enable the counter
        tim.cr1.write(|w| w.cen().enabled());

        Self { tim, pins }
    }

    pub fn release(self) -> (TIM2, PINS) {
        self.tim.cr1.reset();
        (self.tim, self.pins)
    }

    pub fn get_period_clocks(&self) -> u32 {
        self.tim.ccr1.read().ccr().bits()
    }
}
