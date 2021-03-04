//! This file provides an alternative way to set clocks than in the `rcc` modules`,
//! which may be less error prone, and is more opaque. It works by setting
//! scalers etc, then calculating frequencies, instead of solving for a set of scalers
//! that meet specified frequeincies.
//!
//! See STM32CubeIDE for an interactive editor that's very useful for seeing what
//! settings are available, and validating them.
//!
//! See Figure 15 of the Reference Manual for a non-interactive visualization.

use crate::{
    pac::{FLASH, RCC},
    rcc,
    time::U32Ext,
};

/// Speed out of limits.
pub struct SpeedError {}

/// Calculated clock speeds. All in Mhz
#[derive(Clone, Debug)]
pub struct Speeds {
    pub sysclk: f32,
    pub hclk: f32,    // AHB bus, core, memory and DMA
    pub systick: f32, // Cortex System Timer
    pub fclk: f32,    // FCLK Cortex clock
    pub pclk1: f32,   // APB1 peripheral clocks
    pub timer1: f32,  // APB1 timer clocks
    pub pclk2: f32,   // APB2 peripheral clocks
    pub timer2: f32,  // APB2 timer clocks
    pub usb: f32,
    // todo: There are a number of other speeds you could add, like usart1, 3, 5;
    // todo LPUART, I2C etc
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Clk48Src {
    Hsi48 = 0b00, // Only falivd for STM32L49x/L4Ax
    PllSai1 = 0b01,
    Pll = 0b10,
    Msi = 0b11,
}

/// Is a set of speeds valid?
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Validation {
    Valid,
    NotValid,
}

#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Msi(MsiRange),
    Hsi,
    Hse(u8),
}

impl PllSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::None => 0b00,
            Self::Msi(_) => 0b01,
            Self::Hsi => 0b10,
            Self::Hse(_) => 0b11,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum InputSrc {
    Msi(MsiRange),
    Hsi,
    Hse(u8), // freq in Mhz,
    Pll(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Msi(_) => 0b00,
            Self::Hsi => 0b01,
            Self::Hse(_) => 0b10,
            Self::Pll(_) => 0b11,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum MsiRange {
    Range0 = 0b0000,
    Range1 = 0b0001,
    Range2 = 0b0010,
    Range3 = 0b0011,
    Range4 = 0b0100,
    Range5 = 0b0101,
    Range6 = 0b0110,
    Range7 = 0b0111,
    Range8 = 0b1000,
    Range9 = 0b1001,
    Range10 = 0b1010,
    Range11 = 0b1011,
}

impl MsiRange {
    // Calculate the approximate frequency, in Hz.
    fn value(&self) -> u32 {
        match self {
            Self::Range0 => 100_000,
            Self::Range1 => 200_000,
            Self::Range2 => 400_000,
            Self::Range3 => 800_000,
            Self::Range4 => 1_000_000,
            Self::Range5 => 2_000_000,
            Self::Range6 => 4_000_000,
            Self::Range7 => 8_000_000,
            Self::Range8 => 16_000_000,
            Self::Range9 => 24_000_000,
            Self::Range10 => 32_000_000,
            Self::Range11 => 48_000_000,
        }
    }

    /// For backwards compatibility with rcc::MsiFreq.
    fn to_rcc_msi(&self) -> rcc::MsiFreq {
        match self {
            Self::Range0 => rcc::MsiFreq::RANGE100K,
            Self::Range1 => rcc::MsiFreq::RANGE200K,
            Self::Range2 => rcc::MsiFreq::RANGE400K,
            Self::Range3 => rcc::MsiFreq::RANGE800K,
            Self::Range4 => rcc::MsiFreq::RANGE1M,
            Self::Range5 => rcc::MsiFreq::RANGE2M,
            Self::Range6 => rcc::MsiFreq::RANGE4M,
            Self::Range7 => rcc::MsiFreq::RANGE8M,
            Self::Range8 => rcc::MsiFreq::RANGE16M,
            Self::Range9 => rcc::MsiFreq::RANGE24M,
            Self::Range10 => rcc::MsiFreq::RANGE32M,
            Self::Range11 => rcc::MsiFreq::RANGE48M,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// RCC_cfgr2
pub enum Prediv {
    Div1 = 0b0000,
    Div2 = 0b0001,
    Div3 = 0b0010,
    Div4 = 0b0011,
    Div5 = 0b0100,
    Div6 = 0b0101,
    Div7 = 0b0110,
    Div8 = 0b0111,
}

impl Prediv {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div3 => 3,
            Self::Div4 => 4,
            Self::Div5 => 5,
            Self::Div6 => 6,
            Self::Div7 => 7,
            Self::Div8 => 8,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Pllm {
    Div1 = 0b000,
    Div2 = 0b001,
    Div3 = 0b010,
    Div4 = 0b011,
    Div5 = 0b100,
    Div6 = 0b101,
    Div7 = 0b110,
    Div8 = 0b111,
}

impl Pllm {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div3 => 3,
            Self::Div4 => 4,
            Self::Div5 => 5,
            Self::Div6 => 6,
            Self::Div7 => 7,
            Self::Div8 => 8,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
// Main PLL division factor for PLLCLK (system clock
pub enum Pllr {
    Div2 = 0b00,
    Div4 = 0b01,
    Div6 = 0b10,
    Div8 = 0b11,
}

impl Pllr {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div6 => 6,
            Self::Div8 => 8,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Division factor for the AHB clock. Also known as AHB Prescaler.
pub enum HclkPrescaler {
    Div1 = 0b0000,
    Div2 = 0b1000,
    Div4 = 0b1001,
    Div8 = 0b1010,
    Div16 = 0b1011,
    Div64 = 0b1100,
    Div128 = 0b1101,
    Div256 = 0b1110,
    Div512 = 0b1111,
}

impl HclkPrescaler {
    pub fn value(&self) -> u16 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
            Self::Div64 => 64,
            Self::Div128 => 128,
            Self::Div256 => 256,
            Self::Div512 => 512,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// For use with `RCC_APBPPRE1`, and `RCC_APBPPRE2`. Ie, low-speed and high-speed prescalers respectively.
pub enum ApbPrescaler {
    Div1 = 0b000,
    Div2 = 0b100,
    Div4 = 0b101,
    Div8 = 0b110,
    Div16 = 0b111,
}

impl ApbPrescaler {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
        }
    }
}

/// Settings used to configure clocks
pub struct Clocks {
    pub input_src: InputSrc, //
    pub pllm: Pllm,          // PLL divider
    pub pll_vco_mul: u8,     // PLL multiplier. Valid range of 7 to 86.
    pub pll_sai1_mul: u8,    // PLL SAI1 multiplier. Valid range of 7 to 86.
    pub pll_sai2_mul: u8,    // PLL SAI2 multiplier. Valid range of 7 to 86.
    pub pllr: Pllr,
    pub hclk_prescaler: HclkPrescaler, // The AHB clock divider.
    pub apb1_prescaler: ApbPrescaler,  // APB1 divider, for the low speed peripheral bus.
    pub apb2_prescaler: ApbPrescaler,  // APB2 divider, for the high speed peripheral bus.
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
    pub clk48_src: Clk48Src,
    pub sai1_enabled: bool,
    pub sai2_enabled: bool,
    pub hse_bypass: bool,
    pub security_system: bool,
}

impl Clocks {
    /// Setup clocks and return a `Valid` status if the config is valid. Return
    /// `Invalid`, and don't setup if not.
    /// https://docs.rs/stm32f3xx-hal/0.5.0/stm32f3xx_hal/rcc/struct.CFGR.html
    /// Use the STM32CubeIDE Clock Configuration tab to help.
    pub fn setup(&self, rcc: &mut RCC, flash: &mut FLASH) -> Result<(), SpeedError> {
        if let Validation::NotValid = self.validate() {
            return Err(SpeedError {});
        }

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        let (input_freq, sysclk) =
            calc_sysclock(self.input_src, self.pllm, self.pll_vco_mul, self.pllr);

        let hclk = sysclk / self.hclk_prescaler.value() as f32;
        // Reference manual section 3.3.3
        flash.acr.modify(|_, w| unsafe {
            if hclk <= 16. {
                w.latency().bits(0b000)
            } else if hclk <= 32. {
                w.latency().bits(0b001)
            } else if hclk <= 48. {
                w.latency().bits(0b010)
            } else if hclk <= 64. {
                w.latency().bits(0b011)
            } else {
                w.latency().bits(0b100)
            }
        });

        // Reference Manual, 6.2.5:
        // The device embeds 3 PLLs: PLL, PLLSAI1, PLLSAI2. Each PLL provides up to three
        // independent outputs. The internal PLLs can be used to multiply the HSI16, HSE or MSI
        // output clock frequency. The PLLs input frequency must be between 4 and 16 MHz. The
        // selected clock source is divided by a programmable factor PLLM from 1 to 8 to provide a
        // clock frequency in the requested input range. Refer to Figure 15: Clock tree (for
        // STM32L47x/L48x devices) and Figure 16: Clock tree (for STM32L49x/L4Ax devices) and
        // PLL configuration register (RCC_PLLCFGR).
        // The PLLs configuration (selection of the input clock and multiplication factor) must be done
        // before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
        // To modify the PLL configuration, proceed as follows:
        // 1. Disable the PLL by setting PLLON to 0 in Clock control register (RCC_CR).
        // 2. Wait until PLLRDY is cleared. The PLL is now fully stopped.
        // 3. Change the desired parameter.
        // 4. Enable the PLL again by setting PLLON to 1.
        // 5. Enable the desired PLL outputs by configuring PLLPEN, PLLQEN, PLLREN in PLL
        // configuration register (RCC_PLLCFGR).

        // Enable oscillators, and wait until ready.
        match self.input_src {
            InputSrc::Msi(range) => {
                rcc.cr.modify(|_, w| unsafe {
                    w.msirange()
                        .bits(range as u8)
                        .msirgsel()
                        .set_bit()
                        .msion()
                        .set_bit()
                });
                // Wait for the MSI to be ready.
                while rcc.cr.read().msirdy().bit_is_clear() {}
                // todo: If LSE is enabled, calibrate MSI.
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().bit(true));
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy().bit_is_clear() {}
            }
            InputSrc::Hsi => {
                rcc.cr.modify(|_, w| w.hsion().bit(true));
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
            InputSrc::Pll(pll_src) => {
                // todo: PLL setup here is DRY with the HSE, HSI, and MSI setup above.
                match pll_src {
                    PllSrc::Msi(range) => {
                        rcc.cr.modify(|_, w| unsafe {
                            w.msirange()
                                .bits(range as u8)
                                .msirgsel()
                                .set_bit()
                                .msion()
                                .set_bit()
                        });
                    }
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().bit(true));
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi => {
                        rcc.cr.modify(|_, w| w.hsion().bit(true));
                        while rcc.cr.read().hsirdy().bit_is_clear() {}
                    }
                    PllSrc::None => {}
                }
            }
        }

        rcc.cr.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            w.hsebyp().bit(self.hse_bypass)
        });

        if let InputSrc::Pll(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pllrdy().bit_is_set() {}

            rcc.pllcfgr.modify(|_, w| {
                unsafe { w.pllsrc().bits(pll_src.bits()) };
                unsafe { w.plln().bits(self.pll_vco_mul) };
                unsafe { w.pllm().bits(self.pllm as u8) };
                unsafe { w.pllr().bits(self.pllr as u8) }
            });

            if self.sai1_enabled {
                rcc.pllsai1cfgr
                    .modify(|_, w| unsafe { w.pllsai1n().bits(self.pll_sai1_mul) });
            }

            #[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
            if self.sai2_enabled {
                rcc.pllsai2cfgr
                    .modify(|_, w| unsafe { w.pllsai2n().bits(self.pll_sai2_mul) });
            }

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            // todo: Enable sai1 and 2 with separate settings, or lump in with mail PLL
            // like this?
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            if self.sai1_enabled {
                rcc.cr.modify(|_, w| w.pllsai1on().set_bit());
                while rcc.cr.read().pllsai1rdy().bit_is_clear() {}
            }
            #[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
            if self.sai2_enabled {
                rcc.cr.modify(|_, w| w.pllsai2on().set_bit());
                while rcc.cr.read().pllsai2rdy().bit_is_clear() {}
            }

            while rcc.cr.read().pllrdy().bit_is_clear() {}

            // Set Pen, Qen, and Ren after we enable the PLL.
            rcc.pllcfgr.modify(|_, w| {
                w.pllpen().set_bit();
                w.pllqen().set_bit();
                w.pllren().set_bit()
            });

            if self.sai1_enabled {
                rcc.pllsai1cfgr.modify(|_, w| {
                    w.pllsai1pen().set_bit();
                    w.pllsai1qen().set_bit();
                    w.pllsai1ren().set_bit()
                });
            }

            #[cfg(any(feature = "stm32l4x5", feature = "stm32l4x6",))]
            if self.sai2_enabled {
                rcc.pllsai2cfgr.modify(|_, w| {
                    w.pllsai2pen().set_bit();
                    w.pllsai2ren().set_bit()
                });
            }
        }

        rcc.cfgr.modify(|_, w| {
            unsafe { w.sw().bits(self.input_src.bits()) };
            unsafe { w.hpre().bits(self.hclk_prescaler as u8) }; // eg: Divide SYSCLK by 2 to get HCLK of 36Mhz.
            unsafe { w.ppre2().bits(self.apb2_prescaler as u8) }; // HCLK division for APB2.
            unsafe { w.ppre1().bits(self.apb1_prescaler as u8) } // HCLK division for APB1
        });

        rcc.cr.modify(|_, w| w.csson().bit(self.security_system));

        rcc.ccipr
            .modify(|_, w| unsafe { w.clk48sel().bits(self.clk48_src as u8) });

        // Enable the HSI48 as required, which is used for USB, RNG, etc.
        // Only valid for STM32L49x/L4Ax devices.
        if let Clk48Src::Hsi48 = self.clk48_src {
            rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
            while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
        }

        Ok(())
    }

    /// Calculate clock speeds from a given config. Everything is in Mhz.
    /// todo: Handle fractions of mhz. Do floats.
    pub fn calc_speeds(&self) -> Speeds {
        let (input_freq, sysclk) =
            calc_sysclock(self.input_src, self.pllm, self.pll_vco_mul, self.pllr);

        // todo: Is the 2. division at the end of the USB calc always fixed at div2?
        let usb = input_freq as f32 / self.pllm.value() as f32 * self.pll_sai1_mul as f32 / 2.;

        let hclk = sysclk / self.hclk_prescaler.value() as f32;
        let systick = hclk; // todo the required divider is not yet implemented. Either 1x or 8x.(div?)
        let fclk = hclk;
        let pclk1 = hclk / self.apb1_prescaler.value() as f32;
        let timer1 = if let ApbPrescaler::Div1 = self.apb1_prescaler {
            pclk1
        } else {
            pclk1 * 2.
        };
        let pclk2 = hclk / self.apb2_prescaler.value() as f32;
        let timer2 = if let ApbPrescaler::Div1 = self.apb2_prescaler {
            pclk2
        } else {
            pclk2 * 2.
        };

        Speeds {
            sysclk,
            usb,
            hclk,
            systick,
            fclk,
            pclk1,
            timer1,
            pclk2,
            timer2,
        }
    }

    /// Check if valid.
    pub fn validate(&self) -> Validation {
        if self.pll_vco_mul < 7
            || self.pll_vco_mul > 86
            || self.pll_sai1_mul < 7
            || self.pll_sai1_mul > 86
            || self.pll_sai2_mul < 7
            || self.pll_sai2_mul > 86
        {
            return Validation::NotValid;
        }

        validate(self.calc_speeds()).0
    }

    pub fn validate_usb(&self) -> Validation {
        validate(self.calc_speeds()).1
    }

    /// Make a clocks struct from the `rcc` module, that we can pass into existing modules
    /// that use its speeds, like `i2c`, `serial`, `timer` etc.
    pub fn make_rcc_clocks(&self) -> rcc::Clocks {
        let speeds = self.calc_speeds();

        let mut msi = None;
        match self.input_src {
            InputSrc::Msi(range) => {
                msi = Some(range.to_rcc_msi());
            }
            InputSrc::Pll(pll_src) => {
                if let PllSrc::Msi(range) = pll_src {
                    msi = Some(range.to_rcc_msi());
                }
            }
            _ => (),
        }

        let pll_source = match self.input_src {
            InputSrc::Pll(pll_src) => match pll_src {
                PllSrc::Msi(_) => Some(rcc::PllSource::MSI),
                PllSrc::Hsi => Some(rcc::PllSource::HSI16),
                PllSrc::Hse(_) => Some(rcc::PllSource::HSE),
                PllSrc::None => None,
            },
            _ => None,
        };

        rcc::Clocks {
            hclk: (speeds.hclk as u32).mhz().into(),
            hsi48: self.input_src == InputSrc::Hsi,
            msi,
            lsi: false,
            lse: false,
            pclk1: (speeds.pclk1 as u32).mhz().into(),
            pclk2: (speeds.pclk2 as u32).mhz().into(),
            ppre1: self.apb1_prescaler.value(),
            ppre2: self.apb2_prescaler.value(),
            sysclk: (speeds.sysclk as u32).mhz().into(),
            pll_source,
        }
    }

    /// This preset configures clocks with a HSI, a 80Mhz sysclck. All peripheral clocks are at
    /// 80Mhz.
    /// HSE output is not bypassed.
    pub fn hsi_preset() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hsi),
            pllm: Pllm::Div2,
            pll_vco_mul: 20,
            pll_sai1_mul: 8,
            pll_sai2_mul: 8,
            pllr: Pllr::Div2,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div1,
            apb2_prescaler: ApbPrescaler::Div1,
            clk48_src: Clk48Src::PllSai1,
            sai1_enabled: false,
            sai2_enabled: false,
            hse_bypass: false,
            security_system: false,
        }
    }
}

impl Default for Clocks {
    /// This default configures clocks with a HSE, a 32Mhz sysclck. All peripheral clocks are at
    /// 32 Mhz.
    /// HSE output is not bypassed.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hse(8)),
            pllm: Pllm::Div1,
            pll_vco_mul: 20,
            pll_sai1_mul: 8,
            pll_sai2_mul: 8,
            pllr: Pllr::Div2,
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div1,
            apb2_prescaler: ApbPrescaler::Div1,
            clk48_src: Clk48Src::PllSai1,
            sai1_enabled: false,
            sai2_enabled: false,
            hse_bypass: false,
            security_system: false,
        }
    }
}

/// Validate resulting speeds from a given clock config
/// Main validation, USB validation
pub fn validate(speeds: Speeds) -> (Validation, Validation) {
    let mut main = Validation::Valid;
    let mut usb = Validation::Valid;

    // todo: QC these limits
    if speeds.sysclk > 80. || speeds.sysclk < 0. {
        main = Validation::NotValid;
    }

    if speeds.hclk > 80. || speeds.sysclk < 0. {
        main = Validation::NotValid;
    }

    if speeds.pclk1 > 80. || speeds.pclk1 < 0. {
        main = Validation::NotValid;
    }

    if speeds.pclk2 > 80. || speeds.pclk2 < 0. {
        main = Validation::NotValid;
    }

    if speeds.usb as u8 != 48 {
        usb = Validation::NotValid;
    }

    (main, usb)
}

/// Calculate the systick, and input frequency.
fn calc_sysclock(input_src: InputSrc, pllm: Pllm, pll_vco_mul: u8, pllr: Pllr) -> (f32, f32) {
    let input_freq;
    let sysclk = match input_src {
        InputSrc::Pll(pll_src) => {
            input_freq = match pll_src {
                PllSrc::Msi(range) => range.value() as f32 / 1_000_000.,
                PllSrc::Hsi => 16.,
                PllSrc::Hse(freq) => freq as f32,
                PllSrc::None => 0., // todo?
            };
            input_freq as f32 / pllm.value() as f32 * pll_vco_mul as f32 / pllr.value() as f32
        }

        InputSrc::Msi(range) => {
            input_freq = range.value() as f32 / 1_000_000.;
            input_freq
        }
        InputSrc::Hsi => {
            input_freq = 16.;
            input_freq
        }
        InputSrc::Hse(freq) => {
            input_freq = freq as f32;
            input_freq
        }
    };

    (input_freq, sysclk)
}
