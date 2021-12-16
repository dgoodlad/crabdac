use core::marker::PhantomData;

use stm32f4xx_hal::gpio::{Alternate, PushPull};

use crate::hal::pac::TIM2;
use crate::hal::gpio::{gpioa, gpiob};

pub trait ExternalTriggerPin<TIM> {}
impl ExternalTriggerPin<TIM2> for gpioa::PA0<Alternate<PushPull, 1>> {}
impl ExternalTriggerPin<TIM2> for gpioa::PA5<Alternate<PushPull, 1>> {}
impl ExternalTriggerPin<TIM2> for gpioa::PA15<Alternate<PushPull, 1>> {}

pub struct CaptureChannel<TIM, const CH: u8> (PhantomData<TIM>);

impl CaptureChannel<TIM2, 1> {}
impl CaptureChannel<TIM2, 2> {}
impl CaptureChannel<TIM2, 3> {}
impl CaptureChannel<TIM2, 4> {}

#[doc = "ITR1 remap"]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum TIM2_ITR1_RMP_A {
    TIM8_TRGOUT = 0b00,
    PTP_TRGOUT  = 0b01,
    OTG_FS_SOF  = 0b10,
    OTG_HS_SOF  = 0b11,
}
impl From<TIM2_ITR1_RMP_A> for u8 {
    #[inline(always)]
    fn from(variant: TIM2_ITR1_RMP_A) -> Self {
        variant as _
    }
}

pub trait Tim2Itr1Remap<'a>: Sized {
    type W;

    fn variant(self, variant: TIM2_ITR1_RMP_A) -> &'a mut Self::W;

    #[inline(always)]
    fn tim8_trgout(self) -> &'a mut Self::W {
        self.variant(TIM2_ITR1_RMP_A::TIM8_TRGOUT)
    }

    #[inline(always)]
    fn ptp_trgout(self) -> &'a mut Self::W {
        self.variant(TIM2_ITR1_RMP_A::PTP_TRGOUT)
    }

    #[inline(always)]
    fn usb_fs_otg(self) -> &'a mut Self::W {
        self.variant(TIM2_ITR1_RMP_A::OTG_FS_SOF)
    }

    #[inline(always)]
    fn usb_hs_otg(self) -> &'a mut Self::W {
        self.variant(TIM2_ITR1_RMP_A::OTG_HS_SOF)
    }
}

impl<'a> Tim2Itr1Remap<'a> for crate::hal::pac::tim2::or::ITR1_RMP_W<'a> {
    type W = crate::hal::pac::tim2::or::W;

    #[inline(always)]
    fn variant(self, variant: TIM2_ITR1_RMP_A) -> &'a mut Self::W {
        unsafe { self.bits(variant.into()) }
    }
}

pub trait UsbFrameTimer<CH> {
    fn connect_trc(&self);
    fn configure_channel(&self);

    fn enable(&self);
    fn disable(&self);
}

impl<const CH: u8> UsbFrameTimer<CaptureChannel<TIM2, CH>> for TIM2 {
    // Configure the timer to use the OTG_FS_SOF signal as TRC, which is
    // usable as the trigger signal for input capture channels
    fn connect_trc(&self) {
        // Internally map the OTG_FS_SOF signal to ITR1
        //self.or.write(|w| unsafe {
        //    w.itr1_rmp().bits(ITR1_RMP::OTG_FS_SOF as u8)
        //});
        self.or.write(|w| w.itr1_rmp().variant(TIM2_ITR1_RMP_A::OTG_FS_SOF));

        // Set ITR1 as the clock synchronisation trigger, TRC
        self.smcr.write(|w| w.ts().itr1());
    }

    fn configure_channel(&self) {
        match CH {
            1 => {
                self.ccmr1_input().write(|w| w.cc1s().trc());
                self.dier.write(|w| w.cc1ie().enabled());
            },
            2 => {
                self.ccmr1_input().write(|w| w.cc2s().trc());
                self.dier.write(|w| w.cc2ie().enabled());
            },
            3 => {
                self.ccmr2_input().write(|w| w.cc3s().trc());
                self.dier.write(|w| w.cc3ie().enabled());

            },
            4 => {
                self.ccmr2_input().write(|w| w.cc4s().trc());
                self.dier.write(|w| w.cc4ie().enabled());
            },
            0_u8 => { panic!("Invalid channel") }
            5_u8..=u8::MAX => { panic!("Invalid channel") }
        }
    }

    fn enable(&self) {
        self.cr1.write(|w| w.cen().enabled());
    }

    fn disable(&self) {
        self.cr1.write(|w| w.cen().disabled());
    }
}

pub trait ExternallyClockedTimer {
    fn configure_external_clock(&self) {}
}

impl ExternallyClockedTimer for TIM2 {
    fn configure_external_clock(&self) {
        self.smcr.write(|w| { w
            // External Clock Mode 2: TIM2_ETR is used as clock source
            .ece().enabled()
            // without any filter on the clock
            .etf().no_filter()
            // without any prescaler
            .etps().div1()
        });
    }
}

pub struct UsbAudioFrequencyFeedback<TIM, PIN, CH> {
    tim: TIM,
    pin: PIN,
    _ch: PhantomData<CH>,
}

impl<TIM, PIN, CH> UsbAudioFrequencyFeedback<TIM, PIN, CH>
where
    TIM: UsbFrameTimer<CH> + ExternallyClockedTimer,
    PIN: ExternalTriggerPin<TIM>
{
    pub fn new(tim: TIM, pin: PIN) -> Self {
        tim.connect_trc();
        tim.configure_external_clock();
        //tim.configure_channel(channel);

        Self { tim, pin, _ch: PhantomData }
    }

    pub fn start(&self) {
        self.tim.enable();
    }
}
