//! Test the serial interface with the DMA engine
//!
//! This example requires you to short (connect) the TX and RX pins.
#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use(singleton)]
extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
#[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;

extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::cortex_m::asm;
use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::serial::{Config, Serial};
use crate::rt::ExceptionFrame;

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    let channels = p.DMA1.split(&mut rcc.ahb1);
    // let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    // clock configuration using the default settings (all clocks run at 8 MHz)
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
    // TRY this alternate clock configuration (clocks run at nearly the maximum frequency)
    // let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    let mut timer = Delay::new(cp.SYST, clocks);
    // The Serial API is highly generic
    // TRY the commented out, different pin configurations
    let tx = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    // let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);

    let rx = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    // let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);

    // TRY using a different USART peripheral here
    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb2,
    );

    let (mut tx, rx) = serial.split();

    let sent = b'S';

    // The `block!` macro makes an operation block until it finishes
    // NOTE the error type is `!`

    block!(tx.try_write(sent)).ok();

    let buf = singleton!(: [[u8; 8]; 2] = [[0; 8]; 2]).unwrap();

    let mut circ_buffer = rx.circ_read(channels.5, buf);

    // wait for 3 seconds, enter data on serial
    timer.try_delay_ms(1000_u32).ok();
    timer.try_delay_ms(1000_u32).ok();
    timer.try_delay_ms(1000_u32).ok();

    // then view the partial buffer
    let _curr_half = circ_buffer
        .partial_peek(|_buf, half| {
            // do something with _buf here
            Ok((0, half))
        })
        .unwrap();

    asm::bkpt();

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
