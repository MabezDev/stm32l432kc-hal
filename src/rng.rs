extern crate core;
use core::cmp;

use crate::rcc::{Clocks, AHB2};
use crate::stm32::RNG;

/// Extension trait to activate the RNG
pub trait RngExt {
    /// Enables the RNG
    fn enable(self, ahb2: &mut AHB2, clocks: Clocks) -> Rng;
}

impl RngExt for RNG {
    fn enable(self, ahb2: &mut AHB2, clocks: Clocks) -> Rng {
        // crrcr.crrcr().modify(|_, w| w.hsi48on().set_bit()); // p. 180 in ref-manual
        // ...this is now supposed to be done in RCC configuration before freezing

        // hsi48 should be turned on previously or msi at 48mhz
        let msi = match clocks.msi() {
            Some(msi) => msi == crate::rcc::MsiFreq::RANGE48M,
            None => false,
        };
        let hsi = clocks.hsi48();
        assert!(msi || hsi);

        ahb2.enr().modify(|_, w| w.rngen().set_bit());
        // if we don't do this... we can be "too fast", and
        // the following setting of rng.cr.rngen has no effect!!
        while ahb2.enr().read().rngen().bit_is_clear() {}

        self.cr.modify(|_, w| w.rngen().set_bit());

        Rng { rng: self }
    }
}

/// Constrained RNG peripheral
pub struct Rng {
    rng: RNG,
}

impl Rng {
    // cf. https://github.com/nrf-rs/nrf51-hal/blob/master/src/rng.rs#L31
    pub fn free(self) -> RNG {
        // maybe disable the RNG?
        self.rng
    }

    // various methods that are not in the blessed embedded_hal
    // trait list, but may be helpful nonetheless
    // Q: should these be prefixed by underscores?

    pub fn get_random_data(&self) -> u32 {
        while !self.is_data_ready() {}
        self.possibly_invalid_random_data()
        // NB: no need to clear bit here
    }

    // RNG_CR
    /* missing in stm32l4...
    pub fn is_clock_error_detection_enabled(&self) -> bool {
        self.rng.cr.read().ced().bit()
    }
    */

    pub fn is_interrupt_enabled(&self) -> bool {
        self.rng.cr.read().ie().bit()
    }

    pub fn is_enabled(&self) -> bool {
        self.rng.cr.read().rngen().bit()
    }

    // RNG_SR
    pub fn is_clock_error(&self) -> bool {
        self.rng.sr.read().cecs().bit()
    }

    pub fn is_seed_error(&self) -> bool {
        self.rng.sr.read().secs().bit()
    }

    pub fn is_data_ready(&self) -> bool {
        self.rng.sr.read().drdy().bit()
    }

    // RNG_DR
    pub fn possibly_invalid_random_data(&self) -> u32 {
        self.rng.dr.read().rndata().bits()
    }
}

#[derive(Debug)]
pub enum Error {}

impl crate::hal::blocking::rng::Read for Rng {
    // TODO: this error seems pretty useless if it
    // doesn't flag non-enabled RNG or non-started HSI48,
    // but that would be a runtime cost :/
    type Error = Error;

    fn try_read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut i = 0usize;
        while i < buffer.len() {
            let random_word: u32 = self.get_random_data();
            let bytes: [u8; 4] = random_word.to_ne_bytes();
            let n = cmp::min(4, buffer.len() - i);
            buffer[i..i + n].copy_from_slice(&bytes[..n]);
            i += n;
        }

        Ok(())
    }
}
