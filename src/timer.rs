use core::convert::Infallible;

use stm32f4xx_hal::adc::config::ExternalTrigger;
use stm32f4xx_hal::gpio::{Alternate, PushPull};

use crate::hal::pac::TIM2;
use crate::hal::gpio::{gpioa, gpiob};

pub trait ExternalTriggerPin<TIM> {}
impl ExternalTriggerPin<TIM2> for gpioa::PA0<Alternate<PushPull, 1>> {}
impl ExternalTriggerPin<TIM2> for gpioa::PA5<Alternate<PushPull, 1>> {}
impl ExternalTriggerPin<TIM2> for gpioa::PA15<Alternate<PushPull, 1>> {}

pub trait ChannelPin<TIM, const CH: u8> {}

// TODO use features to enumerate these per device type
impl ChannelPin<TIM2, 1> for gpioa::PA0<Alternate<PushPull, 1>> {}
impl ChannelPin<TIM2, 2> for gpioa::PA1<Alternate<PushPull, 1>> {}
impl ChannelPin<TIM2, 3> for gpioa::PA2<Alternate<PushPull, 1>> {}
impl ChannelPin<TIM2, 4> for gpioa::PA3<Alternate<PushPull, 1>> {}

impl ChannelPin<TIM2, 2> for gpiob::PB3<Alternate<PushPull, 1>> {}
impl ChannelPin<TIM2, 3> for gpiob::PB10<Alternate<PushPull, 1>> {}
impl ChannelPin<TIM2, 4> for gpiob::PB11<Alternate<PushPull, 1>> {}

impl ChannelPin<TIM2, 1> for gpioa::PA5<Alternate<PushPull, 1>> {}
impl ChannelPin<TIM2, 1> for gpioa::PA15<Alternate<PushPull, 1>> {}

#[repr(u8)]
enum ITR1_RMP {
    TIM8_TRGOUT = 0b00,
    PTP_TRGOUT  = 0b01,
    OTG_FS_SOF  = 0b10,
    OTG_HS_SOF  = 0b11,
}

#[repr(u8)]
enum CaptureChannel {
    CH1,
    CH2,
    CH3,
    CH4,
}

trait UsbFrameTimer {
    fn connect_trc(&self);
    fn configure_channel(&self, channel: CaptureChannel);

    fn enable(&self);
    fn disable(&self);
}

impl UsbFrameTimer for TIM2 {
    // Configure the timer to use the OTG_FS_SOF signal as TRC, which is
    // usable as the trigger signal for input capture channels
    fn connect_trc(&self) {
        // Internally map the OTG_FS_SOF signal to ITR1
        self.or.write(|w| unsafe {
            w.itr1_rmp().bits(ITR1_RMP::OTG_FS_SOF as u8)
        });

        // Set ITR1 as the clock synchronisation trigger, TRC
        self.smcr.write(|w| { w
            .ts().itr1()
        });
    }

    fn configure_channel(&self, channel: CaptureChannel) {
        match channel {
            CaptureChannel::CH1 => {
                self.ccmr1_input().write(|w| w.cc1s().trc());
                self.dier.write(|w| w.cc1ie().enabled());
            },
            CaptureChannel::CH2 => {
                self.ccmr1_input().write(|w| w.cc2s().trc());
                self.dier.write(|w| w.cc2ie().enabled());
            },
            CaptureChannel::CH3 => {
                self.ccmr2_input().write(|w| w.cc3s().trc());
                self.dier.write(|w| w.cc3ie().enabled());

            },
            CaptureChannel::CH4 => {
                self.ccmr2_input().write(|w| w.cc4s().trc());
                self.dier.write(|w| w.cc4ie().enabled());
            },
        }
    }

    fn enable(&self) {
        self.cr1.write(|w| w.cen().enabled());
    }

    fn disable(&self) {
        self.cr1.write(|w| w.cen().disabled());
    }
}

trait ExternallyClockedTimer {
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

struct UsbAudioFrequencyFeedback<TIM, PIN> {
    tim: TIM,
    pin: PIN,
}

impl<TIM, PIN> UsbAudioFrequencyFeedback<TIM, PIN>
where
    TIM: UsbFrameTimer + ExternallyClockedTimer,
    PIN: ExternalTriggerPin<TIM>
{
    pub fn new(tim: TIM, pin: PIN, channel: CaptureChannel) -> Self {
        tim.connect_trc();
        tim.configure_external_clock();
        tim.configure_channel(channel);

        Self { tim, pin }
    }

    pub fn start(&self) {
        self.tim.enable();
    }
}
