use stm32f4xx_hal::gpio::gpiob::PB8;
use stm32f4xx_hal::gpio::{Alternate, PushPull};
use stm32f4xx_hal::pac::RCC;
use stm32f4xx_hal::rcc::{Enable, Reset, BusTimerClock};

use crate::hal::pac::TIM2;

pub trait UsbFrameTimer {
    fn connect_trc(&self);
    fn configure_channel(&self, channel: CaptureChannel);

    fn enable_counter(&self);
    fn disable_counter(&self);
    fn count(&self) -> Option<u32>;
}

impl UsbFrameTimer for TIM2 {
    // Configure the timer to use the OTG_FS_SOF signal as TRC, which is
    // usable as the trigger signal for input capture channels
    fn connect_trc(&self) {
        self.or.write(|w| w.itr1_rmp().otg_hs_sof());

        // Set ITR1 as the clock synchronisation trigger, TRC
        self.smcr.modify(|_r, w| w
                         .ts().itr1()
                         .sms().reset_mode()
        );

    }

    fn configure_channel(&self, channel: CaptureChannel) {
        self.sr.write(|w| w
                      .cc1if().clear()
                      .tif().clear()
                      .uif().clear()
        );
        match channel {
            CaptureChannel::Channel1 => {
                self.ccmr1_input().modify(|_r, w| w.cc1s().trc());
                self.ccer.write(|w| w.cc1e().set_bit());
                self.dier.write(|w| w.cc1ie().enabled());
            },
            CaptureChannel::Channel2 => {
                self.ccmr1_input().modify(|_r, w| w.cc2s().trc());
                self.ccer.write(|w| w.cc2e().set_bit());
                self.dier.write(|w| w.cc2ie().enabled());
            },
            CaptureChannel::Channel3 => {
                self.ccmr2_input().modify(|_r, w| w.cc3s().trc());
                self.ccer.write(|w| w.cc3e().set_bit());
                self.dier.write(|w| w.cc3ie().enabled());

            },
            CaptureChannel::Channel4 => {
                self.ccmr2_input().modify(|_r, w| w.cc4s().trc());
                self.ccer.write(|w| w.cc4e().set_bit());
                self.dier.write(|w| w.cc4ie().enabled());
            },
        }
        self.egr.write(|w| w.ug().set_bit());
    }

    fn enable_counter(&self) {
        self.sr.modify(|_r, w| w
                      .cc1if().clear()
                      .cc2if().clear()
                      .tif().clear()
                      .uif().clear()
        );
        self.cr1.modify(|_r, w| w.cen().enabled());
    }

    fn disable_counter(&self) {
        self.cr1.write(|w| w.cen().disabled());
    }

    fn count(&self) -> Option<u32> {
        if self.sr.read().cc1if().is_match() {
            self.sr.modify(|_r, w| w.tif().clear());
            Some(self.ccr2.read().bits())
        } else {
            None
        }
    }
}

pub trait ExternallyClockedTimer {
    fn configure_external_clock(&self) {}
}

impl ExternallyClockedTimer for TIM2 {
    fn configure_external_clock(&self) {
        self.smcr.modify(|_r, w| { w
            // External Clock Mode 2: TIM2_ETR is used as clock source
            .ece().enabled()
            // without any filter on the clock
            .etf().no_filter()
            // without any prescaler
            .etps().div1()
        });
    }
}

#[repr(u8)]
pub enum CaptureChannel {
    Channel1,
    Channel2,
    Channel3,
    Channel4,
}

pub trait ExternalTriggerPin<TIM> {}
impl ExternalTriggerPin<TIM2> for PB8<Alternate<PushPull, 1>> {}


pub struct UsbAudioFrequencyFeedback<TIM, PIN> {
    tim: TIM,
    _pin: PIN,
}

impl<TIM, PIN> UsbAudioFrequencyFeedback<TIM, PIN>
where
    TIM: UsbFrameTimer + ExternallyClockedTimer + Enable + Reset + BusTimerClock,
    PIN: ExternalTriggerPin<TIM>,
{
    pub fn new(tim: TIM, channel: CaptureChannel, pin: PIN) -> Self {
        // This unsafe deref is safe as it will only be used for atomic writes
        // to the clock registers
        let rcc = unsafe { &(*RCC::ptr()) };
        TIM::enable(&rcc);
        TIM::reset(&rcc);
        tim.connect_trc();
        tim.configure_external_clock();
        tim.configure_channel(channel);

        Self { tim, _pin: pin }
    }

    pub fn start(&self) {
        self.tim.enable_counter();
    }

    pub fn get_count(&self) -> Option<u32> {
        self.tim.count()
    }
}
