//! Test the Quad SPI interface
//!
//! The example wirtes a command over the QSPI interfaces and recives a 3 byte response.
// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
#[macro_use(entry, exception)]
extern crate cortex_m_rt as rt;
// #[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;

extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

use crate::hal::prelude::*;
use crate::hal::qspi::{Qspi, QspiConfig, QspiMode, QspiReadCommand};
use crate::rt::ExceptionFrame;
use cortex_m::asm;

#[entry]
fn main() -> ! {
    let p = hal::stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut gpioe = p.GPIOE.split(&mut rcc.ahb2);
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);

    // clock configuration (clocks run at nearly the maximum frequency)
    let _ = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let get_id_command = QspiReadCommand {
        instruction: Some((0x9f, QspiMode::SingleChannel)),
        address: None,
        alternative_bytes: None,
        dummy_cycles: 0,
        data_mode: QspiMode::SingleChannel,
        receive_length: 3,
        double_data_rate: false,
    };
    let mut id_arr: [u8; 3] = [0; 3];

    let qspi = {
        let clk = gpioe.pe10.into_af10(&mut gpioe.moder, &mut gpioe.afrh);
        let ncs = gpioe.pe11.into_af10(&mut gpioe.moder, &mut gpioe.afrh);
        let io_0 = gpioe.pe12.into_af10(&mut gpioe.moder, &mut gpioe.afrh);
        let io_1 = gpioe.pe13.into_af10(&mut gpioe.moder, &mut gpioe.afrh);
        let io_2 = gpioe.pe14.into_af10(&mut gpioe.moder, &mut gpioe.afrh);
        let io_3 = gpioe.pe15.into_af10(&mut gpioe.moder, &mut gpioe.afrh);
        Qspi::new(
            p.QUADSPI,
            (clk, ncs, io_0, io_1, io_2, io_3),
            &mut rcc.ahb3,
            QspiConfig::default().clock_prescaler(201),
        ) //Added due to missing OSPEEDR register changes in Qspi
    };

    qspi.transfer(get_id_command, &mut id_arr).unwrap();

    // if all goes well you should reach this breakpoint
    asm::bkpt();

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
