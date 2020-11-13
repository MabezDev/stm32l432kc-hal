//! Example of watchdog timer
#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::time::MilliSeconds;
use crate::hal::watchdog::IndependentWatchdog;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting as sh;
use panic_semihosting as _;
use stm32l4xx_hal as hal;

use crate::sh::hio;
use core::fmt::Write;

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();

    writeln!(hstdout, "Hello, world!").unwrap();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    let clocks = rcc.cfgr.lsi(true).freeze(&mut flash.acr, &mut pwr);

    let mut timer = Delay::new(cp.SYST, clocks);

    // Initiate the independent watchdog timer
    let iwd = IndependentWatchdog::new(dp.IWDG);
    iwd.stop_on_debug(&dp.DBGMCU, true);

    // Start the independent watchdog timer
    if let Ok(mut watchdog) = iwd.try_start(MilliSeconds(1020)) {
        timer.try_delay_ms(1000_u32).ok();

        // Feed the independent watchdog timer
        watchdog.try_feed().ok();
        timer.try_delay_ms(1000_u32).ok();

        // Feed the independent watchdog timer
        watchdog.try_feed().ok();
        timer.try_delay_ms(1000_u32).ok();

        watchdog.try_feed().ok();
        writeln!(hstdout, "Good bye!").unwrap();
    }

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
