//! Blinks an LED

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::hal::datetime::{Date, Time};
use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::rcc::{ClockSecuritySystem, CrystalBypass};
use crate::hal::rtc::{Rtc, RtcClockSource, RtcConfig};
use crate::rt::ExceptionFrame;

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
    let clocks = rcc
        .cfgr
        .lse(CrystalBypass::Disable, ClockSecuritySystem::Disable)
        .freeze(&mut flash.acr, &mut pwr);

    let mut timer = Delay::new(cp.SYST, clocks);

    let mut rtc = Rtc::rtc(
        dp.RTC,
        &mut rcc.apb1r1,
        &mut rcc.bdcr,
        &mut pwr.cr1,
        RtcConfig::default().clock_config(RtcClockSource::LSE),
    );

    let time = Time::new(21.hours(), 57.minutes(), 32.seconds(), 0.micros(), false);
    let date = Date::new(1.day(), 24.date(), 4.month(), 2018.year());

    rtc.set_date_time(date, time);

    timer.try_delay_ms(1000_u32).ok();
    timer.try_delay_ms(1000_u32).ok();
    timer.try_delay_ms(1000_u32).ok();

    let (rtc_date, rtc_time) = rtc.get_date_time();

    writeln!(hstdout, "Time: {:?}", rtc_time).unwrap();
    writeln!(hstdout, "Date: {:?}", rtc_date).unwrap();
    writeln!(hstdout, "Good bye!").unwrap();
    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
