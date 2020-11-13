//! Serial Peripheral Interface (SPI) bus

use core::ptr;

use crate::hal::spi::{FullDuplex, Mode, Phase, Polarity};

use crate::gpio::{Alternate, Floating, Input, AF5};
use crate::rcc::{Clocks, APB1R1, APB2};
use crate::time::Hertz;

/// SPI error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// SCK pin. This trait is sealed and cannot be implemented.
pub trait SckPin<SPI>: private::Sealed {}
/// MISO pin. This trait is sealed and cannot be implemented.
pub trait MisoPin<SPI>: private::Sealed {}
/// MOSI pin. This trait is sealed and cannot be implemented.
pub trait MosiPin<SPI>: private::Sealed {}

macro_rules! pins {
    ($spi:ident, $af:ident, SCK: [$($sck:ident),*], MISO: [$($miso:ident),*], MOSI: [$($mosi:ident),*]) => {
        $(
            impl private::Sealed for $sck<Alternate<$af, Input<Floating>>> {}
            impl SckPin<$spi> for $sck<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $miso<Alternate<$af, Input<Floating>>> {}
            impl MisoPin<$spi> for $miso<Alternate<$af, Input<Floating>>> {}
        )*
        $(
            impl private::Sealed for $mosi<Alternate<$af, Input<Floating>>> {}
            impl MosiPin<$spi> for $mosi<Alternate<$af, Input<Floating>>> {}
        )*
    }
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $APBX:ident, $spiXen:ident, $spiXrst:ident, $pclkX:ident),)+) => {
        $(
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                pub fn $spiX<F>(
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    freq: F,
                    clocks: Clocks,
                    apb2: &mut $APBX,
                ) -> Self
                where
                    F: Into<Hertz>,
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    apb2.enr().modify(|_, w| w.$spiXen().set_bit());
                    apb2.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb2.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                    //        8-bit
                    // DS: 8-bit data size
                    // SSOE: Slave Select output disabled
                    spi.cr2
                        .write(|w| unsafe {
                            w.frxth().set_bit().ds().bits(0b111).ssoe().clear_bit()
                        });

                    let br = Self::compute_baud_rate(clocks.$pclkX(), freq.into());

                    // CPHA: phase
                    // CPOL: polarity
                    // MSTR: master mode
                    // BR: 1 MHz
                    // SPE: SPI disabled
                    // LSBFIRST: MSB first
                    // SSM: enable software slave management (NSS pin free for other uses)
                    // SSI: set nss high = master mode
                    // CRCEN: hardware CRC calculation disabled
                    // BIDIMODE: 2 line unidirectional (full duplex)
                    spi.cr1.write(|w| unsafe {
                        w.cpha()
                            .bit(mode.phase == Phase::CaptureOnSecondTransition)
                            .cpol()
                            .bit(mode.polarity == Polarity::IdleHigh)
                            .mstr()
                            .set_bit()
                            .br()
                            .bits(br)
                            .spe()
                            .set_bit()
                            .lsbfirst()
                            .clear_bit()
                            .ssi()
                            .set_bit()
                            .ssm()
                            .set_bit()
                            .crcen()
                            .clear_bit()
                            .bidimode()
                            .clear_bit()
                    });

                    Spi { spi, pins }
                }

                /// Change the baud rate of the SPI
                pub fn reclock<F>(&mut self, freq: F, clocks: Clocks)
                    where F: Into<Hertz>
                {
                    self.spi.cr1.modify(|_, w| w.spe().clear_bit());
                    self.spi.cr1.modify(|_, w| {
                        unsafe {w.br().bits(Self::compute_baud_rate(clocks.$pclkX(), freq.into()));}
                        w.spe().set_bit()
                    });
                }

                fn compute_baud_rate(clocks: Hertz, freq: Hertz) -> u8 {
                    match clocks.0 / freq.0 {
                        0 => unreachable!(),
                        1..=2 => 0b000,
                        3..=5 => 0b001,
                        6..=11 => 0b010,
                        12..=23 => 0b011,
                        24..=39 => 0b100,
                        40..=95 => 0b101,
                        96..=191 => 0b110,
                        _ => 0b111,
                    }
                }

                /// Releases the SPI peripheral and associated pins
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }
            }

            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn try_read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn try_send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> crate::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::gpio::gpiod::*;
#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
use crate::gpio::gpiog::*;
use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpioe::*};

use crate::stm32::SPI1;
#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
))]
hal! {
    SPI1: (spi1, APB2, spi1en, spi1rst, pclk2),
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
))]
pins!(SPI1, AF5,
    SCK: [PA5, PB3, PE13],
    MISO: [PA6, PB4, PE14],
    MOSI: [PA7, PB5, PE15]);

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6"))]
pins!(SPI1, AF5, SCK: [PG2], MISO: [PG3], MOSI: [PG4]);

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::{gpio::AF6, stm32::SPI3};

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
hal! {
    SPI3: (spi3, APB1R1, spi3en, spi3rst, pclk1),
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
pins!(SPI3, AF6,
    SCK: [PB3, PC10],
    MISO: [PB4, PC11],
    MOSI: [PB5, PC12]);

#[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
pins!(SPI3, AF6, SCK: [PG9], MISO: [PG10], MOSI: [PG11]);

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
use crate::stm32::SPI2;

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
hal! {
    SPI2: (spi2, APB1R1, spi2en, spi2rst, pclk1),
}

#[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6",
))]
pins!(SPI2, AF5,
    SCK: [PB13, PB10, PD1],
    MISO: [PB14, PC2, PD3],
    MOSI: [PB15, PC3, PD4]);
