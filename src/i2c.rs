//! Inter-Integrated Circuit (I2C) bus

use crate::gpio::{Alternate, OpenDrain, Output, AF4};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{Clocks, APB1R1};
#[cfg(feature = "stm32l4x5")]
use crate::stm32::I2C3;
use crate::stm32::{i2c1, I2C1, I2C2};
use crate::time::Hertz;
use cast::u8;
use core::ops::Deref;

const MAX_NBYTE_SIZE: usize = 255;

/// I2C error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK
    Nack,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// SCL pin. This trait is sealed and cannot be implemented.
pub trait SclPin<I2C>: private::Sealed {}

/// SDA pin. This trait is sealed and cannot be implemented.
pub trait SdaPin<I2C>: private::Sealed {}

macro_rules! pins {
    ($spi:ident, $af:ident, SCL: [$($scl:ident),*], SDA: [$($sda:ident),*]) => {
        $(
            impl private::Sealed for $scl<Alternate<$af, Output<OpenDrain>>> {}
            impl SclPin<$spi> for $scl<Alternate<$af, Output<OpenDrain>>> {}
        )*
        $(
            impl private::Sealed for $sda<Alternate<$af, Output<OpenDrain>>> {}
            impl SdaPin<$spi> for $sda<Alternate<$af, Output<OpenDrain>>> {}
        )*
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

/// Start conditions that govern repeated sequential transfer
#[derive(Debug, Clone)]
enum StartCondition {
    /// Non-sequential transfer. Generate both start and stop.
    FirstAndLast,
    /// First transfer in a sequence. Generate start and reload for the next.
    First,
    /// In the middle of sequential transfer. No start/stop generation.
    Middle,
    /// Generate stop to terminate the sequence.
    Last,
}

impl StartCondition {
    /// Configures the I2C sequential transfer
    fn config<'a>(&self, w: &'a mut i2c1::cr2::W) -> &'a mut i2c1::cr2::W {
        use StartCondition::*;
        match self {
            FirstAndLast => w.start().start().autoend().automatic(),
            First => w.start().start().reload().not_completed(),
            Middle => w.start().no_start().reload().not_completed(),
            Last => w.start().no_start().autoend().automatic(),
        }
    }
}

/// Yields start/stop direction from chunks of a payload
#[derive(Debug, Default, Clone)]
struct State {
    total_length: usize,
    current_length: Option<usize>,
}

impl State {
    fn new(total_length: usize) -> Self {
        Self {
            total_length,
            current_length: None,
        }
    }

    fn start_condition(&mut self, chunk_len: usize) -> StartCondition {
        use StartCondition::*;
        let total_len = self.total_length;
        match self.current_length.as_mut() {
            None if chunk_len == total_len => FirstAndLast,
            None => {
                self.current_length.replace(chunk_len);
                First
            }
            Some(len) if *len + chunk_len < total_len => {
                *len += chunk_len;
                Middle
            }
            Some(_) => Last,
        }
    }
}

/// I2C transmission in blocking mode
struct Blocking<'a> {
    cr2: &'a i2c1::CR2,
    isr: &'a i2c1::ISR,
    icr: &'a i2c1::ICR,
}

impl<'a> Blocking<'a> {
    /// Creation succeeds if the I2C interface is not busy. Instead of this
    /// potentially infinite loop, ideally set up a timer and raise a timeout
    /// error.
    fn new(i2c: &'a i2c1::RegisterBlock) -> Result<Self, Error> {
        while i2c.isr.read().busy().is_busy() {}

        Ok(Self {
            cr2: &i2c.cr2,
            isr: &i2c.isr,
            icr: &i2c.icr,
        })
    }

    /// Checks NACK error flag
    fn check_acknowledge_failed(&self) -> Result<(), Error> {
        if self.isr.read().nackf().bit_is_set() {
            // Wait until STOP Flag is reset. AutoEnd should be initiated after
            // AF.
            while self.isr.read().stopf().is_no_stop() {}
            return Err(Error::Nack);
        }
        Ok(())
    }

    /// Waits for a stop detected
    fn wait_on_stop(&mut self) -> Result<(), Error> {
        while self.isr.read().stopf().is_no_stop() {
            self.check_acknowledge_failed()?;
        }
        Ok(())
    }

    /// Waits for a reloaded transfer completed
    fn wait_on_reload(&self) -> Result<(), Error> {
        while self.isr.read().tcr().is_not_complete() {}
        Ok(())
    }
}

/// Normal graceful shutdown procedure
impl<'a> Drop for Blocking<'a> {
    fn drop(&mut self) {
        // Clear STOP flag
        self.icr.write(|w| w.stopcf().clear());
        // Clear configuration register 2
        self.cr2.reset();
    }
}

/// I2C blocking sender for master mode
struct Tx<'a> {
    master: Blocking<'a>,
    txdr: &'a i2c1::TXDR,
    aborted: bool,
}

impl<'a> Tx<'a> {
    fn new(i2c: &'a i2c1::RegisterBlock) -> Result<Self, Error> {
        let master = Blocking::new(i2c)?;

        Ok(Self {
            master,
            txdr: &i2c.txdr,
            aborted: true,
        })
    }

    // Blocks and sends a single byte.
    fn send_byte(&mut self, byte: u8) -> Result<(), Error> {
        while self.master.isr.read().txis().is_not_empty() {
            self.master.check_acknowledge_failed()?;
        }
        // Write data to TXDR
        self.txdr.write(|w| w.txdata().bits(byte));
        Ok(())
    }
}

/// Clean up register state.
impl<'a> Drop for Tx<'a> {
    fn drop(&mut self) {
        if self.aborted {
            // The session was aborted. Register state requires post processing
            // for the error recovery. Clear NACKF flag
            self.master.icr.write(|w| w.nackcf().clear());

            // If a pending TXIS flag is set, write a dummy data in TXDR to
            // clear it
            if self.master.isr.read().txis().is_empty() {
                self.txdr.write(|w| w.txdata().bits(0x00u8));
            }

            // Flush TX register if not empty
            self.master.isr.modify(|r, w| {
                if r.txe().is_not_empty() {
                    w.txe().clear_bit()
                } else {
                    w
                }
            });
        }
    }
}

impl<'a> Write for Tx<'a> {
    type Error = Error;

    fn try_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let total_len = bytes.len();
        bytes
            .chunks(MAX_NBYTE_SIZE)
            .scan(State::new(total_len), |st, chunk| {
                let sc = st.start_condition(chunk.len());
                (sc, chunk).into()
            })
            .try_for_each(|(sc, chunk)| {
                use StartCondition::*;
                if let Middle | Last = sc {
                    self.master.wait_on_reload()?;
                }

                self.master.cr2.write_with_zero(|w| {
                    sc.config(w)
                        .sadd()
                        .bits((addr as u16) << 1)
                        .rd_wrn()
                        .write()
                        .nbytes()
                        .bits(chunk.len() as u8)
                });

                chunk.iter().try_for_each(|byte| self.send_byte(*byte))
            })?;

        self.master.wait_on_stop()?;

        // No error was detected.
        self.aborted = false;
        Ok(())
    }
}

/// I2C blocking receiver for master mode
struct Rx<'a> {
    master: Blocking<'a>,
    rxdr: &'a i2c1::RXDR,
    aborted: bool,
}

impl<'a> Rx<'a> {
    fn new(i2c: &'a i2c1::RegisterBlock) -> Result<Self, Error> {
        let master = Blocking::new(i2c)?;

        Ok(Self {
            master,
            rxdr: &i2c.rxdr,
            aborted: true,
        })
    }

    // Waits for a single byte on this receiver.
    fn recv_byte(&mut self) -> Result<u8, Error> {
        while self.master.isr.read().rxne().is_empty() {
            self.master.check_acknowledge_failed()?;

            // Check if a STOPF is detected
            if self.master.isr.read().stopf().is_stop() {
                if self.master.isr.read().rxne().is_not_empty() {
                    // Reading data from RXDR will be done later.
                    break;
                } else {
                    // TODO: Define and raise a tailored error variant.
                    return Err(Error::Nack);
                }
            }
        }

        // Read data from RXDR
        Ok(self.rxdr.read().rxdata().bits())
    }
}

/// Clean up register state.
impl<'a> Drop for Rx<'a> {
    fn drop(&mut self) {
        if self.aborted {
            // Clear NACKF Flag
            self.master.icr.write(|w| w.nackcf().clear());
        }
    }
}

impl<'a> Read for Rx<'a> {
    type Error = Error;

    fn try_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let total_len = buffer.len();
        buffer
            .chunks_mut(MAX_NBYTE_SIZE)
            .scan(State::new(total_len), |st, chunk| {
                let sc = st.start_condition(chunk.len());
                (sc, chunk).into()
            })
            .try_for_each(|(sc, chunk)| {
                use StartCondition::*;
                if let Middle | Last = sc {
                    self.master.wait_on_reload()?;
                }

                self.master.cr2.write_with_zero(|w| {
                    sc.config(w)
                        .sadd()
                        .bits((addr as u16) << 1)
                        .rd_wrn()
                        .read()
                        .nbytes()
                        .bits(chunk.len() as u8)
                });

                chunk
                    .iter_mut()
                    .try_for_each(|byte| self.recv_byte().map(|v| *byte = v))
            })?;

        self.master.wait_on_stop()?;

        // No error was detected.
        self.aborted = false;
        Ok(())
    }
}

impl<SCL, SDA> I2c<I2C1, (SCL, SDA)> {
    pub fn i2c1<F>(i2c: I2C1, pins: (SCL, SDA), freq: F, clocks: Clocks, apb1: &mut APB1R1) -> Self
    where
        F: Into<Hertz>,
        SCL: SclPin<I2C1>,
        SDA: SdaPin<I2C1>,
    {
        apb1.enr().modify(|_, w| w.i2c1en().set_bit());
        apb1.rstr().modify(|_, w| w.i2c1rst().set_bit());
        apb1.rstr().modify(|_, w| w.i2c1rst().clear_bit());
        Self::new(i2c, pins, freq, clocks)
    }
}

impl<SCL, SDA> I2c<I2C2, (SCL, SDA)> {
    pub fn i2c2<F>(i2c: I2C2, pins: (SCL, SDA), freq: F, clocks: Clocks, apb1: &mut APB1R1) -> Self
    where
        F: Into<Hertz>,
        SCL: SclPin<I2C2>,
        SDA: SdaPin<I2C2>,
    {
        apb1.enr().modify(|_, w| w.i2c2en().set_bit());
        apb1.rstr().modify(|_, w| w.i2c2rst().set_bit());
        apb1.rstr().modify(|_, w| w.i2c2rst().clear_bit());
        Self::new(i2c, pins, freq, clocks)
    }
}

impl<SCL, SDA, I2C> I2c<I2C, (SCL, SDA)>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    /// Configures the I2C peripheral to work in master mode
    fn new<F>(i2c: I2C, pins: (SCL, SDA), freq: F, clocks: Clocks) -> Self
    where
        F: Into<Hertz>,
        SCL: SclPin<I2C>,
        SDA: SdaPin<I2C>,
    {
        let freq = freq.into().0;
        assert!(freq <= 1_000_000);
        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // TODO review compliance with the timing requirements of I2C
        // t_I2CCLK = 1 / PCLK1
        // t_PRESC  = (PRESC + 1) * t_I2CCLK
        // t_SCLL   = (SCLL + 1) * t_PRESC
        // t_SCLH   = (SCLH + 1) * t_PRESC
        //
        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
        let i2cclk = clocks.pclk1().0;
        let ratio = i2cclk / freq - 4;
        let (presc, scll, sclh, sdadel, scldel) = if freq >= 100_000 {
            // fast-mode or fast-mode plus
            // here we pick SCLL + 1 = 2 * (SCLH + 1)
            let presc = ratio / 387;

            let sclh = ((ratio / (presc + 1)) - 3) / 3;
            let scll = 2 * (sclh + 1) - 1;

            let (sdadel, scldel) = if freq > 400_000 {
                // fast-mode plus
                let sdadel = 0;
                let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            } else {
                // fast-mode
                let sdadel = i2cclk / 8_000_000 / (presc + 1);
                let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            };

            (presc, scll, sclh, sdadel, scldel)
        } else {
            // standard-mode
            // here we pick SCLL = SCLH
            let presc = ratio / 514;

            let sclh = ((ratio / (presc + 1)) - 2) / 2;
            let scll = sclh;

            let sdadel = i2cclk / 2_000_000 / (presc + 1);
            let scldel = i2cclk / 800_000 / (presc + 1) - 1;

            (presc, scll, sclh, sdadel, scldel)
        };

        let presc = u8(presc).unwrap();
        assert!(presc < 16);
        let scldel = u8(scldel).unwrap();
        assert!(scldel < 16);
        let sdadel = u8(sdadel).unwrap();
        assert!(sdadel < 16);
        let sclh = u8(sclh).unwrap();
        let scll = u8(scll).unwrap();

        // Configure for "fast mode" (400 KHz)
        i2c.timingr.write(|w| {
            w.presc()
                .bits(presc)
                .scll()
                .bits(scll)
                .sclh()
                .bits(sclh)
                .sdadel()
                .bits(sdadel)
                .scldel()
                .bits(scldel)
        });

        // Enable the peripheral
        i2c.cr1.write(|w| w.pe().set_bit());

        I2c { i2c, pins }
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, (SCL, SDA)) {
        (self.i2c, self.pins)
    }
}

impl<PINS, I2C> Write for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn try_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        Tx::new(&self.i2c)?.try_write(addr, bytes)
    }
}

impl<PINS, I2C> Read for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn try_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        Rx::new(&self.i2c)?.try_read(addr, buffer)
    }
}

impl<PINS, I2C> WriteRead for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn try_write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        self.try_write(addr, bytes)?;
        self.try_read(addr, buffer)
    }
}

use crate::gpio::gpioa::{PA10, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};

#[cfg(feature = "stm32l4x5")]
use crate::gpio::gpioc::{PC0, PC1};

pins!(I2C1, AF4,
    SCL: [PA9, PB6],
    SDA: [PA10, PB7]);

pins!(I2C2, AF4, SCL: [PB10], SDA: [PB11]);

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
use crate::gpio::gpiob::{PB13, PB14, PB8, PB9};

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
pins!(I2C1, AF4, SCL: [PB8], SDA: [PB9]);

#[cfg(any(feature = "stm32l4x1", feature = "stm32l4x6"))]
pins!(I2C2, AF4, SCL: [PB13], SDA: [PB14]);

#[cfg(feature = "stm32l4x5")]
pins!(I2C3, AF4, SCL: [PC0], SDA: [PC1]);
