//! # Analog to Digital converter

use core::convert::Infallible;
use core::ptr;

use crate::{
    gpio::Analog,
    hal::{
        adc::{Channel as EmbeddedHalChannel, OneShot},
        blocking::delay::DelayUs,
    },
    pac,
    rcc::{AHB2, CCIPR},
};

// TODO: Only on STM32L47x/L48x.
use crate::gpio::AnalogPin;

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x2", feature = "stm32l4x3",))]
use pac::ADC as ADC1;

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
use pac::ADC1;

/// Analog to Digital converter interface
pub struct ADC {
    inner: ADC1,
    resolution: Resolution,
    sample_time: SampleTime,
}

impl ADC {
    /// Initialize the ADC
    pub fn new(
        inner: ADC1,
        ahb: &mut AHB2,
        ccipr: &mut CCIPR,
        delay: &mut impl DelayUs<u32>,
    ) -> Self {
        // Enable peripheral
        ahb.enr().modify(|_, w| w.adcen().set_bit());

        // Reset peripheral
        ahb.rstr().modify(|_, w| w.adcrst().set_bit());
        ahb.rstr().modify(|_, w| w.adcrst().clear_bit());

        // Select system clock as ADC clock source
        ccipr.ccipr().modify(|_, w| {
            // This is sound, as `0b11` is a valid value for this field.
            unsafe {
                w.adcsel().bits(0b11);
            }

            w
        });

        // Initialize the ADC, according to the STM32L4xx Reference Manual,
        // section 16.4.6.
        inner.cr.write(|w| w.deeppwd().clear_bit()); // exit deep-power-down mode
        inner.cr.modify(|_, w| w.advregen().set_bit()); // enable internal voltage regulator

        // According to the STM32L4xx Reference Manual, section 16.4.6, we need
        // to wait for T_ADCVREG_STUP after enabling the internal voltage
        // regulator. For the STM32L433, this is 20 us. We choose 25 us to
        // account for bad clocks.
        delay.delay_us(25);

        // Calibration procedure according to section 16.4.8.
        inner.cr.modify(|_, w| {
            w.adcal().set_bit(); // start calibration
            w.adcaldif().clear_bit(); // single-ended mode

            w
        });

        while inner.cr.read().adcal().bit_is_set() {}

        // We need to wait 4 ADC clock after ADCAL goes low, 1 us is more than enough
        delay.delay_us(1);

        Self {
            inner,
            resolution: Resolution::default(),
            sample_time: SampleTime::default(),
        }
    }

    /// Set the ADC resolution
    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.resolution = resolution;
    }

    /// Set the sample time
    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    /// Release the ADC peripheral
    ///
    /// Drops `ADC` and returns the `pac::ADC` that is was wrapping, giving the
    /// user full access to the peripheral.
    pub fn release(self) -> ADC1 {
        self.inner
    }
}

// TODO: AnalogPin only on STM32L47x/L48x.
impl<C: AnalogPin> OneShot<ADC, u16, C> for ADC
where
    C: Channel,
{
    type Error = Infallible;

    fn read(&mut self, channel: &mut C) -> nb::Result<u16, Self::Error> {
        // Make sure bits are off
        while self.inner.cr.read().addis().bit_is_set() {}

        // Enable ADC
        self.inner.isr.write(|w| w.adrdy().set_bit());
        self.inner.cr.modify(|_, w| w.aden().set_bit());
        while self.inner.isr.read().adrdy().bit_is_clear() {}

        // Configure ADC
        self.inner.cfgr.write(|w| {
            // This is sound, as all `Resolution` values are valid for this
            // field.
            unsafe { w.res().bits(self.resolution as u8) }
        });

        // Configure channel
        channel.set_sample_time(&self.inner, self.sample_time);

        // TODO: Only on STM32L47x/L48x.
        // Connect the pin to the ADC
        channel.connect_adc();

        // Select channel
        self.inner.sqr1.write(|w| {
            // This is sound, as all `Channel` implementations set valid values.
            unsafe {
                w.sq1().bits(C::channel());
            }

            w
        });

        // Start conversion
        self.inner
            .isr
            .modify(|_, w| w.eos().set_bit().eoc().set_bit());
        self.inner.cr.modify(|_, w| w.adstart().set_bit());
        while self.inner.isr.read().eos().bit_is_clear() {}

        // Read ADC value first time and discard it, as per errata sheet.
        // The errata states that if we do conversions slower than 1 kHz, the
        // first read ADC value can be corrupted, so we discard it and measure again.
        let _ = unsafe { ptr::read_volatile(&self.inner.dr.read().bits()) };

        self.inner
            .isr
            .modify(|_, w| w.eos().set_bit().eoc().set_bit());
        self.inner.cr.modify(|_, w| w.adstart().set_bit());
        while self.inner.isr.read().eos().bit_is_clear() {}

        // Read ADC value
        let val = self.inner.dr.read().bits() as u16;

        // TODO: Only on STM32L47x/L48x.
        // Disconnect the pin from the ADC
        channel.disconnect_adc();

        // Disable ADC
        self.inner.cr.modify(|_, w| w.addis().set_bit());

        Ok(val)
    }
}

/// ADC resolution setting
///
/// The default setting is 12 bits.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Resolution {
    /// 12-bit resolution
    Bits12 = 0b00,

    /// 10-bit resolution
    Bits10 = 0b01,

    /// 8-bit resolution
    Bits8 = 0b10,

    /// 6-bit resolution
    Bits6 = 0b11,
}

impl Default for Resolution {
    fn default() -> Self {
        Self::Bits12
    }
}

/// ADC sample time
///
/// The default setting is 2.5 ADC clock cycles.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum SampleTime {
    /// 2.5 ADC clock cycles
    Cycles2_5 = 0b000,

    /// 6.5 ADC clock cycles
    Cycles6_5 = 0b001,

    /// 12.5 ADC clock cycles
    Cycles12_5 = 0b010,

    /// 24.5 ADC clock cycles
    Cycles24_5 = 0b011,

    /// 47.5 ADC clock cycles
    Cycles47_5 = 0b100,

    /// 92.5 ADC clock cycles
    Cycles92_5 = 0b101,

    /// 247.5 ADC clock cycles
    Cycles247_5 = 0b110,

    /// 640.5 ADC clock cycles
    Cycles640_5 = 0b111,
}

impl Default for SampleTime {
    fn default() -> Self {
        Self::Cycles2_5
    }
}

/// Implemented for all types that represent ADC channels
pub trait Channel: EmbeddedHalChannel<ADC, ID = u8> {
    fn set_sample_time(&mut self, adc: &ADC1, sample_time: SampleTime);
}

macro_rules! external_channels {
    (
        $(
            $id:expr,
            $pin:ident,
            $smpr:ident,
            $smp:ident;
        )*
    ) => {
        $(
            impl EmbeddedHalChannel<ADC> for crate::gpio::$pin<Analog> {
                type ID = u8;

                fn channel() -> Self::ID {
                    $id
                }
            }

            impl Channel for crate::gpio::$pin<Analog> {
                fn set_sample_time(&mut self,
                    adc: &ADC1,
                    sample_time: SampleTime,
                ) {
                    adc.$smpr.modify(|_, w| {
                        // This is sound, as all `SampleTime` values are valid
                        // for this field.
                        unsafe {
                            w.$smp().bits(sample_time as u8)
                        }
                    })
                }
            }
        )*
    };
}

external_channels!(
    1,  PC0, smpr1, smp1;
    2,  PC1, smpr1, smp2;
    3,  PC2, smpr1, smp3;
    4,  PC3, smpr1, smp4;
    5,  PA0, smpr1, smp5;
    6,  PA1, smpr1, smp6;
    7,  PA2, smpr1, smp7;
    8,  PA3, smpr1, smp8;
    9,  PA4, smpr1, smp9;
    10, PA5, smpr2, smp10;
    11, PA6, smpr2, smp11;
    12, PA7, smpr2, smp12;
    13, PC4, smpr2, smp13;
    14, PC5, smpr2, smp14;
    15, PB0, smpr2, smp15;
    16, PB1, smpr2, smp16;
);
