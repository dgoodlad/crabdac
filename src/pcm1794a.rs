use core::convert::Infallible;

use embedded_hal_one::digital::{blocking::*, PinState, ErrorType};
use embedded_hal_one::digital::PinState::{Low, High};

#[derive(Debug)]
pub enum Error {
    Unsupported,
    InvalidConfiguration,
}

pub trait Pins {
    fn rst(&mut self, reset: PinState) -> Result<(), Error>;
    fn mono(&mut self, mono: PinState) -> Result<(), Error>;
    fn chsl(&mut self, chsl: PinState) -> Result<(), Error>;
    fn dem(&mut self, dem: PinState) -> Result<(), Error>;
    fn mute(&mut self, mute: PinState) -> Result<(), Error>;
    fn fmt0(&mut self, fmt0: PinState) -> Result<(), Error>;
    fn fmt1(&mut self, fmt1: PinState) -> Result<(), Error>;
    fn zero(&self) -> Result<PinState, Error>;
}

impl<RST, MONO, CHSL, MUTE, FMT0, FMT1> Pins for (RST, MONO, CHSL, MUTE, FMT0, FMT1)
where
    RST: OutputPin + ErrorType<Error = Infallible>,
    MONO: OutputPin + ErrorType<Error = Infallible>,
    CHSL: OutputPin + ErrorType<Error = Infallible>,
    MUTE: OutputPin + ErrorType<Error = Infallible>,
    FMT0: OutputPin + ErrorType<Error = Infallible>,
    FMT1: OutputPin + ErrorType<Error = Infallible>,
{
    fn rst(&mut self, rst: PinState) -> Result<(), Error> {
        self.0.set_state(PinState::from(rst)).unwrap();
        Ok(())
    }

    fn mono(&mut self, mono: PinState) -> Result<(), Error> {
        self.1.set_state(PinState::from(mono)).unwrap();
        Ok(())
    }

    fn chsl(&mut self, chsl: PinState) -> Result<(), Error> {
        self.2.set_state(PinState::from(chsl)).unwrap();
        Ok(())
    }

    fn dem(&mut self, _dem: PinState) -> Result<(), Error> {
        Err(Error::Unsupported)
    }

    fn mute(&mut self, mute: PinState) -> Result<(), Error> {
        self.3.set_state(PinState::from(mute)).unwrap();
        Ok(())
    }

    fn fmt0(&mut self, fmt0: PinState) -> Result<(), Error> {
        self.4.set_state(PinState::from(fmt0)).unwrap();
        Ok(())
    }

    fn fmt1(&mut self, fmt1: PinState) -> Result<(), Error> {
        self.5.set_state(PinState::from(fmt1)).unwrap();
        Ok(())
    }

    fn zero(&self) -> Result<PinState, Error> {
        Err(Error::Unsupported)
    }
}

impl<RST, MONO, CHSL, DEM, MUTE, FMT0, FMT1, ZERO> Pins for (RST, MONO, CHSL, DEM, MUTE, FMT0, FMT1, ZERO)
where
    RST: OutputPin + ErrorType<Error = Infallible>,
    MONO: OutputPin + ErrorType<Error = Infallible>,
    CHSL: OutputPin + ErrorType<Error = Infallible>,
    DEM: OutputPin + ErrorType<Error = Infallible>,
    MUTE: OutputPin + ErrorType<Error = Infallible>,
    FMT0: OutputPin + ErrorType<Error = Infallible>,
    FMT1: OutputPin + ErrorType<Error = Infallible>,
    ZERO: InputPin + ErrorType<Error = Infallible>,
{
    fn rst(&mut self, rst: PinState) -> Result<(), Error> {
        self.0.set_state(PinState::from(rst)).unwrap();
        Ok(())
    }

    fn mono(&mut self, mono: PinState) -> Result<(), Error> {
        self.1.set_state(PinState::from(mono)).unwrap();
        Ok(())
    }

    fn chsl(&mut self, chsl: PinState) -> Result<(), Error> {
        self.2.set_state(PinState::from(chsl)).unwrap();
        Ok(())
    }

    fn dem(&mut self, dem: PinState) -> Result<(), Error> {
        self.3.set_state(PinState::from(dem)).unwrap();
        Ok(())
    }

    fn mute(&mut self, mute: PinState) -> Result<(), Error> {
        self.4.set_state(PinState::from(mute)).unwrap();
        Ok(())
    }

    fn fmt0(&mut self, fmt0: PinState) -> Result<(), Error> {
        self.5.set_state(PinState::from(fmt0)).unwrap();
        Ok(())
    }

    fn fmt1(&mut self, fmt1: PinState) -> Result<(), Error> {
        self.6.set_state(PinState::from(fmt1)).unwrap();
        Ok(())
    }

    fn zero(&self) -> Result<PinState, Error> {
        Ok(self.7.is_high().unwrap().into())
    }
}

/// Audio data format
pub enum Format {
    /// I2S Philips
    I2S,
    /// Left Justified aka MSB Justified
    LeftJustified,
    /// Standard 16-bit aka LSB Justified 16-bit
    Lsb16,
    /// Standard 24-bit aka LSB Justified 24-bit
    Lsb24,
}

/// Channel configuration
pub enum Channels {
    /// Monaural, left-channel only
    MonoL,
    /// Monaural, right-channel only
    MonoR,
    /// Stereo
    Stereo
}

/// Digital Filter Rolloff
pub enum DfRolloff {
    Sharp,
    Slow,
    None,
}

/// PCM1794A by Texas Instruments (nee Burr-Brown) Driver
///
/// This device is controlled by a number of simple digital signal inputs,
/// unlike its cousin PCM1792A which includes a serial interface.
///
/// You should configure some GPIO pins
///
///     let rst = pb12.into();
///     let mono = pa1.into();
///     ...
///
///     let dac = Pcm1794a::new((rst, mono, chsl, mute, fmt0, fmt1));
///     dac.configure_and_reset(Format::I2s, Channels::Stereo, DfRolloff::Sharp);

pub struct Pcm1794a<P: Pins> {
    pins: P,
}

impl<P: Pins> Pcm1794a<P> {
    pub fn new(pins: P) -> Self {
        Self { pins }
    }

    /// Reset the DAC until the given function returns
    pub fn reset<F, T>(&mut self, f: F) -> T where F: FnOnce(&mut Self) -> T {
        self.pin_rst(Low).unwrap();
        let result = f(self);
        self.pin_rst(High).unwrap();
        result
    }

    pub fn enable(&mut self) -> Result<(), Error> {
        self.pin_rst(High)
    }

    /// "Safely" configure the DAC by muting first, putting it into reset,
    /// changing its configuration, then letting it start up again.
    ///
    /// *Note* the chip will be left muted, and it is up to you to unmute!
    pub fn safely_configure(&mut self, format: Format, channels: Channels, df_rolloff: DfRolloff) -> Result<(), Error> {
        self.mute().unwrap();
        self.reset(|x| x.configure(format, channels, df_rolloff))
    }

    /// Change the DAC's configuration "live". This may lead to undesired audio
    /// artifacts.
    pub fn configure(&mut self, format: Format, channels: Channels, df_rolloff: DfRolloff) -> Result<(), Error> {
        // See PCM1794A datasheet Table 3. Audio Data Format Select
        let (mono, chsl, fmt1, fmt0) = match (format, channels, df_rolloff) {
            (Format::I2S, Channels::Stereo, DfRolloff::Sharp) => (Low, Low, Low, Low),
            (Format::LeftJustified, Channels::Stereo, DfRolloff::Sharp) => (Low, Low, Low, High),
            (Format::Lsb16, Channels::Stereo, DfRolloff::Sharp) => (Low, Low, High, Low),
            (Format::Lsb24, Channels::Stereo, DfRolloff::Sharp) => (Low, Low, High, High),

            (Format::I2S, Channels::Stereo, DfRolloff::Slow) => (Low, High, Low, Low),
            (Format::LeftJustified, Channels::Stereo, DfRolloff::Slow) => (Low, High, Low, High),
            (Format::Lsb16, Channels::Stereo, DfRolloff::Slow) => (Low, High, High, Low),
            (Format::Lsb24, Channels::Stereo, DfRolloff::None) => (Low, High, High, High),

            (Format::I2S, Channels::MonoL, DfRolloff::Sharp) => (High, Low, Low, Low),
            (Format::LeftJustified, Channels::MonoL, DfRolloff::Sharp) => (High, Low, Low, High),
            (Format::Lsb16, Channels::MonoL, DfRolloff::Sharp) => (High, Low, High, Low),
            (Format::Lsb24, Channels::MonoL, DfRolloff::Sharp) => (High, Low, High, High),

            (Format::I2S, Channels::MonoR, DfRolloff::Sharp) => (High, High, Low, Low),
            (Format::LeftJustified, Channels::MonoR, DfRolloff::Sharp) => (High, High, Low, High),
            (Format::Lsb16, Channels::MonoR, DfRolloff::Sharp) => (High, High, High, Low),
            (Format::Lsb24, Channels::MonoR, DfRolloff::Sharp) => (High, High, High, High),

            _ => return Err(Error::InvalidConfiguration),
        };

        self.pin_mono(mono).unwrap();
        self.pin_chsl(chsl).unwrap();
        self.pin_fmt1(fmt1).unwrap();
        self.pin_fmt0(fmt0).unwrap();

        Ok(())
    }

    /// Soft-mute the DAC's output, pop-free.
    ///
    /// Per the datasheet:
    ///
    /// > both analog outputs transition to the bipolar zero level in -0.5dB
    /// > steps with a transition speed of 1/f_s per step.
    pub fn mute(&mut self) -> Result<(), Error> {
        self.pin_mute(High)
    }

    /// Unmute the DAC (see mute())
    pub fn unmute(&mut self) -> Result<(), Error> {
        self.pin_mute(Low)
    }

    /// Mute or unmute the DAC (see mute())
    pub fn set_mute(&mut self, mute: bool) -> Result<(), Error> {
        self.pin_mute(mute.into())
    }

    /// Enable the De-Emphasis filter
    ///
    /// This filter is only applicable if the sampling rate is 44.1 kHz!
    pub fn set_deemphasis(&mut self, dem: bool) -> Result<(), Error> {
        self.pin_dem(dem.into())
    }

    fn pin_rst(&mut self, rst: PinState) -> Result<(), Error> { self.pins.rst(rst) }
    fn pin_mono(&mut self, mono: PinState) -> Result<(), Error> { self.pins.mono(mono) }
    fn pin_chsl(&mut self, chsl: PinState) -> Result<(), Error> { self.pins.chsl(chsl) }
    fn pin_dem(&mut self, dem: PinState) -> Result<(), Error> { self.pins.dem(dem) }
    fn pin_mute(&mut self, mute: PinState) -> Result<(), Error> { self.pins.mute(mute) }
    fn pin_fmt0(&mut self, fmt0: PinState) -> Result<(), Error> { self.pins.fmt0(fmt0) }
    fn pin_fmt1(&mut self, fmt1: PinState) -> Result<(), Error> { self.pins.fmt1(fmt1) }
    fn pin_zero(&self) -> Result<PinState, Error> { self.pins.zero() }
}
