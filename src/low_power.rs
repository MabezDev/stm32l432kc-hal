//! This module contains code used to place the STM32L4 in low power modes.
//! Reference section 5.3.3: `Low power modes` of the Reference Manual.

use crate::pac::{PWR, RCC};
use cortex_m::{asm::wfi, peripheral::SCB};

// These enums are better suited for a clocks or rcc module.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PllSrc {
    Msi = 0b00, // todo: check bit values
    Hsi16 = 0b01,
    Hse = 0b10,
}

#[derive(Clone, Copy)]
pub enum InputSrc {
    Msi,
    Hsi16,
    Hse,
    Pll(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Msi => 0b00, // todo check bit values
            Self::Hsi16 => 0b00,
            Self::Hse => 0b01,
            Self::Pll(_) => 0b10,
        }
    }
}

// See L4 Reference Manual section 5.3.6. The values correspond
// todo PWR_CR1, LPMS field.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum StopMode {
    Zero = 0b000,
    One = 0b001,
    Two = 0b010,
}

/// Re-select innput source; used on Stop and Standby modes, where the system reverts
/// to HSI after wake.
fn re_select_input(input_src: InputSrc) {
    // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

    // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
    // todo: But this saves a few reg writes.
    match input_src {
        InputSrc::Hse => unsafe {
            (*RCC::ptr()).cr.modify(|_, w| w.hseon().set_bit());
            while (*RCC::ptr()).cr.read().hserdy().bit_is_clear() {}

            (*RCC::ptr())
                .cfgr
                .modify(|_, w| w.sw().bits(input_src.bits()));
        },
        InputSrc::Pll(_) => unsafe {
            // todo: DRY with above.
            (*RCC::ptr()).cr.modify(|_, w| w.hseon().set_bit());
            while (*RCC::ptr()).cr.read().hserdy().bit_is_clear() {}

            (*RCC::ptr()).cr.modify(|_, w| w.pllon().clear_bit());
            while (*RCC::ptr()).cr.read().pllrdy().bit_is_set() {}

            (*RCC::ptr())
                .cfgr
                .modify(|_, w| w.sw().bits(input_src.bits()));

            (*RCC::ptr()).cr.modify(|_, w| w.pllon().set_bit());
            while (*RCC::ptr()).cr.read().pllrdy().bit_is_clear() {}
        },
        InputSrc::Hsi16 => (), // Already reset to this? todo
        InputSrc::Msi => (),   // Already reset to this? todo
    }
}

/// Ref man, table 24
/// Note that this assumes you've already reduced clock frequency below 2 Mhz.
pub fn low_power_run(pwr: &mut PWR) {
    // Decrease the system clock frequency below 2 MHz
    // LPR = 1
    pwr.cr1.modify(|_, w| w.lpr().set_bit())
}

/// Ref man, table 24
/// Return to normal run mode from low-power run. Requires you to increase the clock speed
/// manually after running this.
pub fn return_from_low_power_run(pwr: &mut PWR) {
    // LPR = 0
    pwr.cr1.modify(|_, w| w.lpr().clear_bit());

    // Wait until REGLPF = 0
    while pwr.sr2.read().reglpf().bit_is_set() {}

    // Increase the system clock frequency
}

/// Place the system in sleep now mode. To enter `low-power sleep now`, enter low power mode
/// (eg `low_power_mode()`) before running this. Ref man, table 25 and 26
pub fn sleep_now(scb: &mut SCB) {
    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:
    // – SLEEPDEEP = 0
    // – No interrupt (for WFI) or event (for WFE) is pending
    scb.clear_sleepdeep();

    // Or, unimplemented:
    // On return from ISR while:
    // // SLEEPDEEP = 0 and SLEEPONEXIT = 1
    // scb.clear_sleepdeep();
    // scb.set_sleeponexit();

    wfi();
}

/// Enter Stop 0, Stop 1, or Stop 2 modes. Reference manual, section 5.3.6. Tables 27, 28, and 29.
pub fn stop(scb: &mut SCB, pwr: &mut PWR, mode: StopMode, input_src: InputSrc) {
    // WFI (Wait for Interrupt) or WFE (Wait for Event) while:
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    scb.set_sleepdeep();
    // – No interrupt (for WFI) or event (for WFE) is pending
    // – LPMS = (according to mode) in PWR_CR1
    pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(mode as u8) });

    // Or, unimplemented:
    // On Return from ISR while:
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    // – SLEEPONEXIT = 1
    // – No interrupt is pending
    // – LPMS = “000” in PWR_CR1

    wfi();

    re_select_input(input_src);
}

/// Enter `Standby` mode. See
/// Table 30.
pub fn standby(scb: &mut SCB, pwr: &mut PWR, input_src: InputSrc) {
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    scb.set_sleepdeep();
    // – No interrupt (for WFI) or event (for WFE) is pending
    // – LPMS = “011” in PWR_CR1
    pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(0b011) });
    // – WUFx bits are cleared in power status register 1 (PWR_SR1)
    // (Clear by setting cwfuf bits in `pwr_scr`.)
    pwr.scr.write(|w| unsafe { w.bits(0) });
    // todo: Unsure why setting the individual bits isn't working; PWR.scr doesn't have modify method?
    // pwr.scr.modify(|_, w| {
    //     w.cwuf1().set_bit();
    //     w.cwuf2().set_bit();
    //     w.cwuf3().set_bit();
    //     w.cwuf4().set_bit();
    //     w.cwuf5().set_bit();
    // })

    // Or, unimplemented:
    // On return from ISR while:
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    // – SLEEPONEXIT = 1
    // – No interrupt is pending
    // – LPMS = “011” in PWR_CR1 and
    // – WUFx bits are cleared in power status register 1 (PWR_SR1)
    // – The RTC flag corresponding to the chosen wakeup source (RTC Alarm
    // A, RTC Alarm B, RTC wakeup, tamper or timestamp flags) is cleared
    wfi();

    re_select_input(input_src);
}

/// Enter `Shutdown mode` mode: the lowest-power of the 3 low-power states avail. See
/// Table 31.
pub fn shutdown(scb: &mut SCB, pwr: &mut PWR, input_src: InputSrc) {
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    scb.set_sleepdeep();
    // – No interrupt (for WFI) or event (for WFE) is pending
    // – LPMS = “011” in PWR_CR1
    pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(0b100) });
    // – WUFx bits are cleared in power status register 1 (PWR_SR1)
    // (Clear by setting cwfuf bits in `pwr_scr`.)
    pwr.scr.write(|w| unsafe { w.bits(0) });
    // todo: Unsure why setting the individual bits isn't working; PWR.scr doesn't have modify method?
    // pwr.scr.modify(|_, w| {
    //     w.cwuf1().set_bit();
    //     w.cwuf2().set_bit();
    //     w.cwuf3().set_bit();
    //     w.cwuf4().set_bit();
    //     w.cwuf5().set_bit();
    // })

    // Or, unimplemented:
    // On return from ISR while:
    // – SLEEPDEEP bit is set in Cortex®-M4 System Control register
    // – SLEEPONEXT = 1
    // – No interrupt is pending
    // – LPMS = “1XX” in PWR_CR1 and
    // – WUFx bits are cleared in power status register 1 (PWR_SR1)
    // – The RTC flag corresponding to the chosen wakeup source (RTC
    // Alarm A, RTC Alarm B, RTC wakeup, tamper or timestamp flags) is
    // cleared
    wfi();

    re_select_input(input_src);
}
