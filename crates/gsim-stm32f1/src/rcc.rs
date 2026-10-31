//! Reset and clock control (RCC).
//!
//! The model implements the full STM32F103 clock tree: HSI/HSE/PLL with
//! realistic start-up delays, the AHB/APB prescaler tables, the timer and ADC
//! clock multipliers, per-peripheral gating bits and the peripheral reset
//! registers. Firmware that polls `RCC_CR`/`RCC_CFGR` like the HAL does sees
//! the same ready bits and switch timings as on silicon.

use super::map::rcc as reg;
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::{Clocks, Ctx};
use gsim_core::irq::lines;

/// Internal high speed RC oscillator.
pub const HSI_HZ: u32 = 8_000_000;
/// Internal low speed RC oscillator (watchdog / RTC clock).
pub const LSI_HZ: u32 = 40_000;
/// External low speed crystal used by the RTC.
pub const LSE_HZ: u32 = 32_768;
/// Crystal frequency assumed on OSC_IN until the board configures another one.
pub const DEFAULT_HSE_HZ: u32 = 8_000_000;

/// HSI start-up time (datasheet: 2 us).
pub const HSI_STARTUP_NS: u64 = 2_000;
/// HSE start-up time (8 MHz crystal, ~2 ms).
pub const HSE_STARTUP_NS: u64 = 2_000_000;
/// PLL lock time.
pub const PLL_LOCK_NS: u64 = 200_000;
/// LSI start-up time.
pub const LSI_STARTUP_NS: u64 = 40_000;
/// LSE start-up time (32 kHz crystal).
pub const LSE_STARTUP_NS: u64 = 2_000_000_000;

/// `CR` bits.
pub mod cr {
    pub const HSION: u32 = 1 << 0;
    pub const HSIRDY: u32 = 1 << 1;
    pub const HSITRIM: u32 = 0x1F << 3;
    pub const HSEON: u32 = 1 << 16;
    pub const HSERDY: u32 = 1 << 17;
    pub const HSEBYP: u32 = 1 << 18;
    pub const CSSON: u32 = 1 << 19;
    pub const PLLON: u32 = 1 << 24;
    pub const PLLRDY: u32 = 1 << 25;
}

/// `CFGR` sysclk source codes.
pub mod sw {
    pub const HSI: u32 = 0;
    pub const HSE: u32 = 1;
    pub const PLL: u32 = 2;
}

/// `CIR` bits.
pub mod cir {
    pub const LSIRDYF: u32 = 1 << 0;
    pub const LSERDYF: u32 = 1 << 1;
    pub const HSIRDYF: u32 = 1 << 2;
    pub const HSERDYF: u32 = 1 << 3;
    pub const PLLRDYF: u32 = 1 << 4;
    pub const CSSF: u32 = 1 << 7;
    pub const LSIRDYIE: u32 = 1 << 8;
    pub const LSERDYIE: u32 = 1 << 9;
    pub const HSIRDYIE: u32 = 1 << 10;
    pub const HSERDYIE: u32 = 1 << 11;
    pub const PLLRDYIE: u32 = 1 << 12;
    pub const LSIRDYC: u32 = 1 << 16;
    pub const LSERDYC: u32 = 1 << 17;
    pub const HSIRDYC: u32 = 1 << 18;
    pub const HSERDYC: u32 = 1 << 19;
    pub const PLLRDYC: u32 = 1 << 20;
    pub const CSSC: u32 = 1 << 23;
}

/// `BDCR` bits.
pub mod bdcr {
    pub const LSEON: u32 = 1 << 0;
    pub const LSERDY: u32 = 1 << 1;
    pub const LSEBYP: u32 = 1 << 2;
    pub const RTCSEL: u32 = 0x3 << 8;
    pub const RTCEN: u32 = 1 << 15;
    pub const BDRST: u32 = 1 << 16;

    /// RTC clock source selection values.
    pub const RTCSEL_NONE: u32 = 0;
    pub const RTCSEL_LSE: u32 = 1;
    pub const RTCSEL_LSI: u32 = 2;
    pub const RTCSEL_HSE: u32 = 3;
}

/// `CSR` bits.
pub mod csr {
    pub const LSION: u32 = 1 << 0;
    pub const LSIRDY: u32 = 1 << 1;
    pub const RMVF: u32 = 1 << 24;
    pub const PINRSTF: u32 = 1 << 26;
    pub const PORRSTF: u32 = 1 << 27;
    pub const SFTRSTF: u32 = 1 << 28;
    pub const IWDGRSTF: u32 = 1 << 29;
    pub const WWDGRSTF: u32 = 1 << 30;
    pub const LPWRRSTF: u32 = 1 << 31;
}

const CR_WRITABLE: u32 = cr::HSION | cr::HSITRIM | cr::HSEON | cr::HSEBYP | cr::CSSON | cr::PLLON;
const CFGR_WRITABLE: u32 = 0x077F_FFF3;
const AHBENR_WRITABLE: u32 = 0x0557;
const APB2ENR_WRITABLE: u32 = 0x0038_FFFD;
/// `APB1ENR` gates of the STM32F103 medium density parts (RM0008 8.3.8):
/// timers 2..4, WWDG, SPI2/3, USART2/3, I2C1/2, USB, CAN1, BKP and PWR.
const APB1ENR_WRITABLE: u32 = 0x1AFE_8C3F;
const BDCR_WRITABLE: u32 = 0x0001_8305;
const CSR_WRITABLE: u32 = csr::LSION | csr::RMVF;

/// Oscillator whose ready flag is pending.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Startup {
    Hsi,
    Hse,
    Pll,
    Lsi,
    Lse,
}

/// RCC state.
#[derive(Debug, Clone)]
pub struct Rcc {
    cr: u32,
    cfgr: u32,
    cir: u32,
    apb2rstr: u32,
    apb1rstr: u32,
    ahbenr: u32,
    apb2enr: u32,
    apb1enr: u32,
    bdcr: u32,
    csr: u32,
    sws: u32,
    hsi_ready: bool,
    hse_ready: bool,
    pll_ready: bool,
    lsi_ready: bool,
    lse_ready: bool,
    pending: Vec<(Startup, u64)>,
    hse_hz: u32,
    irq_high: bool,
    reset_requests: Option<(u32, u32)>,
    backup_reset: bool,
}

impl Default for Rcc {
    fn default() -> Self {
        Rcc::new()
    }
}

impl Rcc {
    pub fn new() -> Self {
        Rcc {
            cr: cr::HSION,
            cfgr: 0,
            cir: 0,
            apb2rstr: 0,
            apb1rstr: 0,
            ahbenr: 0,
            apb2enr: 0,
            apb1enr: 0,
            bdcr: 0,
            // Power-on reset is the reason for the initial reset.
            csr: csr::PORRSTF,
            sws: sw::HSI,
            hsi_ready: true,
            hse_ready: false,
            pll_ready: false,
            lsi_ready: false,
            lse_ready: false,
            pending: Vec::new(),
            hse_hz: DEFAULT_HSE_HZ,
            irq_high: false,
            reset_requests: None,
            backup_reset: false,
        }
    }

    /// Restores the power-on state, including the `PORRSTF` flag.
    pub fn reset(&mut self) {
        let hse_hz = self.hse_hz;
        *self = Rcc::new();
        self.hse_hz = hse_hz;
    }

    /// Configures the crystal attached to OSC_IN/OSC_OUT.
    pub fn set_hse_hz(&mut self, hz: u32) {
        self.hse_hz = hz;
    }

    pub fn hse_hz(&self) -> u32 {
        self.hse_hz
    }

    fn schedule(&mut self, what: Startup, cx: &Ctx, delay_ns: u64) {
        let at = cx.now_ns() + delay_ns;
        self.pending.retain(|(item, _)| *item != what);
        self.pending.push((what, at));
    }

    fn set_ready(&mut self, what: Startup, cx: &Ctx) {
        match what {
            Startup::Hsi => {
                self.hsi_ready = true;
                self.cir |= cir::HSIRDYF;
            }
            Startup::Hse => {
                self.hse_ready = true;
                self.cir |= cir::HSERDYF;
                self.maybe_start_pll(cx);
            }
            Startup::Pll => {
                self.pll_ready = true;
                self.cir |= cir::PLLRDYF;
            }
            Startup::Lsi => {
                self.lsi_ready = true;
                self.cir |= cir::LSIRDYF;
            }
            Startup::Lse => {
                self.lse_ready = true;
                self.cir |= cir::LSERDYF;
            }
        }
    }

    fn maybe_start_pll(&mut self, cx: &Ctx) {
        if self.cr & cr::PLLON == 0 || self.pll_ready {
            return;
        }
        let source_ready = if self.cr_source_is_hse() {
            self.hse_ready
        } else {
            self.hsi_ready
        };
        if source_ready {
            self.schedule(Startup::Pll, cx, PLL_LOCK_NS);
        }
    }

    fn cr_source_is_hse(&self) -> bool {
        self.cfgr & (1 << 16) != 0
    }

    /// PLL output frequency, before the system clock prescaler.
    pub fn pll_hz(&self) -> u32 {
        let input = if self.cr_source_is_hse() {
            if self.cfgr & (1 << 17) != 0 {
                self.hse_hz / 2
            } else {
                self.hse_hz
            }
        } else {
            HSI_HZ / 2
        };
        let multiplier = match field(self.cfgr, 18, 4) {
            0b1110 | 0b1111 => 16,
            other => other + 2,
        };
        input * multiplier
    }

    fn ahb_divider(&self) -> u32 {
        const TABLE: [u32; 8] = [2, 4, 8, 16, 64, 128, 256, 512];
        let hpre = field(self.cfgr, 4, 4);
        if hpre < 8 {
            1
        } else {
            TABLE[(hpre - 8) as usize]
        }
    }

    fn apb1_divider(&self) -> u32 {
        let ppre = field(self.cfgr, 8, 3);
        if ppre < 4 {
            1
        } else {
            1 << (ppre - 3)
        }
    }

    fn apb2_divider(&self) -> u32 {
        let ppre = field(self.cfgr, 11, 3);
        if ppre < 4 {
            1
        } else {
            1 << (ppre - 3)
        }
    }

    /// Selected system clock source frequency.
    pub fn sysclk_hz(&self) -> u32 {
        match self.sws {
            sw::HSE => self.hse_hz,
            sw::PLL => self.pll_hz(),
            _ => HSI_HZ,
        }
    }

    /// Full clock tree snapshot for peripherals.
    pub fn clocks(&self) -> Clocks {
        let sysclk = self.sysclk_hz();
        let hclk = sysclk / self.ahb_divider();
        let apb1_div = self.apb1_divider();
        let apb2_div = self.apb2_divider();
        let pclk1 = hclk / apb1_div;
        let pclk2 = hclk / apb2_div;
        let adc_div = match field(self.cfgr, 14, 2) {
            0 => 2,
            1 => 4,
            2 => 6,
            _ => 8,
        };
        Clocks {
            sysclk_hz: sysclk,
            hclk_hz: hclk,
            pclk1_hz: pclk1,
            pclk2_hz: pclk2,
            tim1_hz: if apb2_div == 1 { pclk2 } else { pclk2 * 2 },
            tim2_hz: if apb1_div == 1 { pclk1 } else { pclk1 * 2 },
            adc_hz: pclk2 / adc_div,
            rtc_hz: self.rtc_clock_hz(),
        }
    }

    /// Clock feeding a peripheral on APB2, or `0` when the gate is closed.
    pub fn apb2_clock(&self, gate: u32) -> u32 {
        if self.apb2enr & gate != 0 {
            self.clocks().pclk2_hz
        } else {
            0
        }
    }

    /// Clock feeding a peripheral on APB1, or `0` when the gate is closed.
    pub fn apb1_clock(&self, gate: u32) -> u32 {
        if self.apb1enr & gate != 0 {
            self.clocks().pclk1_hz
        } else {
            0
        }
    }

    /// Clock feeding an APB1 timer (`TIM1` lives on APB2).
    pub fn apb1_timer_clock(&self, gate: u32) -> u32 {
        if self.apb1enr & gate != 0 {
            self.clocks().tim2_hz
        } else {
            0
        }
    }

    /// Clock feeding `TIM1`.
    pub fn tim1_clock(&self) -> u32 {
        if self.apb2enr & (1 << 11) != 0 {
            self.clocks().tim1_hz
        } else {
            0
        }
    }

    /// Clock feeding the ADC prescaler output.
    pub fn adc_clock(&self, gate: u32) -> u32 {
        if self.apb2enr & gate != 0 {
            self.clocks().adc_hz
        } else {
            0
        }
    }

    /// Clock feeding a DMA controller.
    pub fn dma_clock(&self, gate: u32) -> u32 {
        if self.ahbenr & gate != 0 {
            self.clocks().hclk_hz
        } else {
            0
        }
    }

    /// RTC clock frequency derived from `BDCR.RTCSEL`.
    pub fn rtc_clock_hz(&self) -> u32 {
        if self.bdcr & bdcr::RTCEN == 0 {
            return 0;
        }
        match field(self.bdcr, 8, 2) {
            bdcr::RTCSEL_LSE => LSE_HZ,
            bdcr::RTCSEL_LSI => LSI_HZ,
            bdcr::RTCSEL_HSE => self.hse_hz / 128,
            _ => 0,
        }
    }

    /// True while the LSE/HSE oscillator that clocks the RTC is stable.
    pub fn rtc_clock_ready(&self) -> bool {
        match field(self.bdcr, 8, 2) {
            bdcr::RTCSEL_LSE => self.lse_ready,
            bdcr::RTCSEL_LSI => self.lsi_ready,
            bdcr::RTCSEL_HSE => self.hse_ready,
            _ => false,
        }
    }

    pub fn lsi_hz(&self) -> u32 {
        LSI_HZ
    }

    pub fn hse_ready(&self) -> bool {
        self.hse_ready
    }

    pub fn pll_locked(&self) -> bool {
        self.pll_ready
    }

    pub fn system_clock_source(&self) -> u32 {
        self.sws
    }

    /// `MCO` pin frequency.
    pub fn mco_hz(&self) -> u32 {
        let clocks = self.clocks();
        match field(self.cfgr, 24, 3) {
            0b100 => clocks.sysclk_hz,
            0b101 => clocks.hclk_hz,
            0b110 => clocks.pclk1_hz,
            0b111 => clocks.pclk2_hz,
            _ => 0,
        }
    }

    /// Takes the pending peripheral reset request, if a reset register was hit.
    pub fn take_reset_requests(&mut self) -> Option<(u32, u32)> {
        self.reset_requests.take()
    }

    /// True when `BDCR.BDRST` was written, which resets RTC and backup registers.
    pub fn take_backup_reset(&mut self) -> bool {
        std::mem::take(&mut self.backup_reset)
    }

    /// Records a reset cause, used by the SoC reset handler.
    pub fn set_reset_flag(&mut self, flag: u32) {
        self.csr |= flag;
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let flags = self.cir & 0x1F;
        let enables = (self.cir >> 8) & 0x1F;
        let high = flags & enables != 0;
        if high != self.irq_high {
            self.irq_high = high;
            cx.irq(lines::RCC, high);
        }
    }

    pub fn advance(&mut self, cx: &Ctx, _elapsed_ns: u64) {
        let now = cx.now_ns();
        let mut fired = Vec::new();
        self.pending.retain(|(what, at)| {
            if *at <= now {
                fired.push(*what);
                false
            } else {
                true
            }
        });
        for what in fired {
            self.set_ready(what, cx);
        }

        // The system clock follows SW as soon as the requested source is ready.
        let requested = field(self.cfgr, 0, 2);
        if requested != self.sws {
            let ready = match requested {
                sw::HSI => self.hsi_ready,
                sw::HSE => self.hse_ready,
                sw::PLL => self.pll_ready,
                _ => false,
            };
            if ready {
                self.sws = requested;
                cx.note("RCC", "system clock switched", requested);
            }
        }

        self.refresh_irq(cx);
    }

    fn write_cr(&mut self, cx: &Ctx, value: u32) {
        let old = self.cr;
        self.cr = (old & !CR_WRITABLE) | (value & CR_WRITABLE);

        if self.cr & cr::HSION != 0 && !self.hsi_ready && old & cr::HSION == 0 {
            self.schedule(Startup::Hsi, cx, HSI_STARTUP_NS);
        }
        if self.cr & cr::HSION == 0 {
            self.hsi_ready = false;
        }
        if self.cr & cr::HSEON != 0 && !self.hse_ready && old & cr::HSEON == 0 {
            self.schedule(Startup::Hse, cx, HSE_STARTUP_NS);
        }
        if self.cr & cr::HSEON == 0 {
            self.hse_ready = false;
            self.pll_ready = false;
        }
        if self.cr & cr::PLLON != 0 {
            self.maybe_start_pll(cx);
        } else {
            self.pll_ready = false;
        }
    }

    fn write_cir(&mut self, cx: &Ctx, value: u32) {
        let enables =
            value & (cir::LSIRDYIE | cir::LSERDYIE | cir::HSIRDYIE | cir::HSERDYIE | cir::PLLRDYIE);
        self.cir = (self.cir & !0x1F00) | enables;
        let clears =
            value & (cir::LSIRDYC | cir::LSERDYC | cir::HSIRDYC | cir::HSERDYC | cir::PLLRDYC);
        if clears & cir::LSIRDYC != 0 {
            self.cir &= !cir::LSIRDYF;
        }
        if clears & cir::LSERDYC != 0 {
            self.cir &= !cir::LSERDYF;
        }
        if clears & cir::HSIRDYC != 0 {
            self.cir &= !cir::HSIRDYF;
        }
        if clears & cir::HSERDYC != 0 {
            self.cir &= !cir::HSERDYF;
        }
        if clears & cir::PLLRDYC != 0 {
            self.cir &= !cir::PLLRDYF;
        }
        if value & cir::CSSC != 0 {
            self.cir &= !cir::CSSF;
        }
        self.refresh_irq(cx);
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::CR => {
                let mut value = self.cr;
                if self.hsi_ready {
                    value |= cr::HSIRDY;
                }
                if self.hse_ready {
                    value |= cr::HSERDY;
                }
                if self.pll_ready {
                    value |= cr::PLLRDY;
                }
                value
            }
            reg::CFGR => (self.cfgr & !0xC) | (self.sws << 2),
            reg::CIR => self.cir,
            reg::APB2RSTR => self.apb2rstr,
            reg::APB1RSTR => self.apb1rstr,
            reg::AHBENR => self.ahbenr,
            reg::APB2ENR => self.apb2enr,
            reg::APB1ENR => self.apb1enr,
            reg::BDCR => self.bdcr | if self.lse_ready { bdcr::LSERDY } else { 0 },
            reg::CSR => self.csr | if self.lsi_ready { csr::LSIRDY } else { 0 },
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: false,
                })
            }
        };
        Ok(extract(value, offset, width))
    }

    pub fn write_reg(
        &mut self,
        cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        match offset & !3 {
            reg::CR => {
                let merged = merge(self.cr, offset, width, value);
                self.write_cr(cx, merged);
            }
            reg::CFGR => {
                let merged = merge(self.cfgr, offset, width, value) & CFGR_WRITABLE;
                self.cfgr = (self.cfgr & !0x0FFF_FFFF) | merged;
            }
            reg::CIR => self.write_cir(cx, merge(self.cir, offset, width, value)),
            reg::APB2RSTR => {
                self.apb2rstr = merge(self.apb2rstr, offset, width, value);
                if self.apb2rstr != 0 {
                    let (apb1, apb2) = self.reset_requests.unwrap_or((0, 0));
                    self.reset_requests = Some((apb1, apb2 | self.apb2rstr));
                }
            }
            reg::APB1RSTR => {
                self.apb1rstr = merge(self.apb1rstr, offset, width, value);
                if self.apb1rstr != 0 {
                    let (apb1, apb2) = self.reset_requests.unwrap_or((0, 0));
                    self.reset_requests = Some((apb1 | self.apb1rstr, apb2));
                }
            }
            reg::AHBENR => self.ahbenr = merge(self.ahbenr, offset, width, value) & AHBENR_WRITABLE,
            reg::APB2ENR => {
                self.apb2enr = merge(self.apb2enr, offset, width, value) & APB2ENR_WRITABLE
            }
            reg::APB1ENR => {
                self.apb1enr = merge(self.apb1enr, offset, width, value) & APB1ENR_WRITABLE
            }
            reg::BDCR => {
                let merged = merge(self.bdcr, offset, width, value);
                if merged & bdcr::LSEON != 0 && !self.lse_ready && self.bdcr & bdcr::LSEON == 0 {
                    self.schedule(Startup::Lse, cx, LSE_STARTUP_NS);
                }
                if merged & bdcr::LSEON == 0 {
                    self.lse_ready = false;
                }
                if merged & bdcr::BDRST != 0 {
                    self.backup_reset = true;
                }
                self.bdcr = (self.bdcr & !BDCR_WRITABLE) | (merged & BDCR_WRITABLE & !bdcr::BDRST);
            }
            reg::CSR => {
                let merged = merge(self.csr, offset, width, value);
                if merged & csr::LSION != 0 && !self.lsi_ready && self.csr & csr::LSION == 0 {
                    self.schedule(Startup::Lsi, cx, LSI_STARTUP_NS);
                }
                if merged & csr::LSION == 0 {
                    self.lsi_ready = false;
                }
                if merged & csr::RMVF != 0 {
                    self.csr &= !(csr::PINRSTF
                        | csr::PORRSTF
                        | csr::SFTRSTF
                        | csr::IWDGRSTF
                        | csr::WWDGRSTF
                        | csr::LPWRRSTF);
                }
                self.csr = (self.csr & !CSR_WRITABLE) | (merged & csr::LSION);
            }
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: true,
                })
            }
        }
        Ok(())
    }
}

crate::impl_peripheral!(Rcc, "RCC");

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::HSI_8MHZ)
    }

    fn write(rcc: &mut Rcc, cx: &Ctx, offset: u32, value: u32) {
        rcc.write_reg(cx, offset, AccessWidth::Word, value).unwrap();
    }

    fn read(rcc: &mut Rcc, cx: &Ctx, offset: u32) -> u32 {
        rcc.read_reg(cx, offset, AccessWidth::Word).unwrap()
    }

    #[test]
    fn power_on_clock_is_hsi() {
        let mut rcc = Rcc::new();
        let cx = ctx();
        assert_eq!(read(&mut rcc, &cx, reg::CR) & cr::HSIRDY, cr::HSIRDY);
        assert_eq!(rcc.sysclk_hz(), HSI_HZ);
        assert_eq!(rcc.clocks().hclk_hz, HSI_HZ);
    }

    #[test]
    fn pll_startup_takes_time_and_reaches_72mhz() {
        let mut rcc = Rcc::new();
        let at = |ns: u64| Ctx::new(ns, Clocks::HSI_8MHZ);

        // HSE on, start-up takes about 2 ms.
        write(&mut rcc, &at(0), reg::CR, cr::HSION | cr::HSEON);
        assert!(!rcc.hse_ready());
        rcc.advance(&at(1_000_000), 0);
        assert!(!rcc.hse_ready());
        rcc.advance(&at(2_100_000), 0);
        assert!(rcc.hse_ready());
        assert_eq!(
            read(&mut rcc, &at(2_100_000), reg::CR) & cr::HSERDY,
            cr::HSERDY
        );

        // PLL on, x9, HSE source: lock is scheduled when PLLON is written.
        write(
            &mut rcc,
            &at(2_100_000),
            reg::CFGR,
            (0b0111 << 18) | (1 << 16),
        );
        write(
            &mut rcc,
            &at(2_100_000),
            reg::CR,
            cr::HSION | cr::HSEON | cr::PLLON,
        );
        rcc.advance(&at(2_200_000), 0);
        assert!(!rcc.pll_locked(), "PLL is still locking");
        rcc.advance(&at(2_400_000), 0);
        assert!(rcc.pll_locked());
        assert_eq!(rcc.pll_hz(), 72_000_000);

        // Switch the system clock to the PLL.
        write(
            &mut rcc,
            &at(2_400_000),
            reg::CFGR,
            (0b0111 << 18) | (1 << 16) | sw::PLL,
        );
        rcc.advance(&at(2_500_000), 0);
        assert_eq!(rcc.system_clock_source(), sw::PLL);
        let clocks = rcc.clocks();
        assert_eq!(clocks.sysclk_hz, 72_000_000);
        assert_eq!(clocks.hclk_hz, 72_000_000);
        assert_eq!(clocks.pclk1_hz, 72_000_000);
        assert_eq!(clocks.tim2_hz, 72_000_000);
    }

    #[test]
    fn apb_prescalers_follow_rm0008_table() {
        let mut rcc = Rcc::new();
        let cx = ctx();
        // HPRE = /1 (0b0000), PPRE1 = /2 (0b100), PPRE2 = /1, ADCPRE = /6.
        write(&mut rcc, &cx, reg::CFGR, (0b100 << 8) | (0b10 << 14));
        let clocks = rcc.clocks();
        assert_eq!(clocks.hclk_hz, HSI_HZ);
        assert_eq!(clocks.pclk1_hz, HSI_HZ / 2);
        assert_eq!(clocks.pclk2_hz, HSI_HZ);
        assert_eq!(clocks.tim2_hz, HSI_HZ, "APB1 timer clock is doubled");
        assert_eq!(clocks.tim1_hz, HSI_HZ);
        assert_eq!(clocks.adc_hz, HSI_HZ / 6);

        // HPRE = /512 (0b1111)
        write(&mut rcc, &cx, reg::CFGR, 0b1111 << 4);
        assert_eq!(rcc.clocks().hclk_hz, HSI_HZ / 512);
    }

    #[test]
    fn peripheral_gating_drives_clock_output() {
        let mut rcc = Rcc::new();
        let cx = ctx();
        assert_eq!(rcc.apb2_clock(1 << 14), 0);
        write(&mut rcc, &cx, reg::APB2ENR, 1 << 14);
        assert_eq!(rcc.apb2_clock(1 << 14), HSI_HZ);
        assert_eq!(rcc.apb1_clock(1 << 17), 0);
    }

    #[test]
    fn reset_request_register_reports_touched_peripherals() {
        let mut rcc = Rcc::new();
        let cx = ctx();
        write(&mut rcc, &cx, reg::APB2RSTR, 1 << 14);
        assert_eq!(rcc.take_reset_requests(), Some((0, 1 << 14)));
        assert_eq!(rcc.take_reset_requests(), None);
    }

    #[test]
    fn ready_interrupt_reaches_the_nvic_line() {
        let mut rcc = Rcc::new();
        let start = ctx();
        write(&mut rcc, &start, reg::CIR, cir::HSERDYIE);
        write(&mut rcc, &start, reg::CR, cr::HSION | cr::HSEON);
        let later = Ctx::new(3_000_000, Clocks::HSI_8MHZ);
        rcc.advance(&later, 0);
        let events = later.take_events();
        assert!(
            events.iter().any(
                |e| matches!(e, gsim_core::Event::Irq { irq, high: true } if *irq == lines::RCC)
            ),
            "expected the RCC interrupt to be raised, got {events:?}"
        );
    }
}

impl Rcc {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self) -> Vec<(&'static str, String)> {
        let clocks = self.clocks();
        let source = match field(self.cfgr, 2, 2) {
            0 => "HSI",
            1 => "HSE",
            2 => "PLL",
            _ => "保留",
        };
        vec![
            ("时钟源", source.to_string()),
            ("SYSCLK", crate::status::hz(clocks.sysclk_hz)),
            ("HCLK", crate::status::hz(clocks.hclk_hz)),
            ("PCLK1", crate::status::hz(clocks.pclk1_hz)),
            ("PCLK2", crate::status::hz(clocks.pclk2_hz)),
            ("ADC 时钟", crate::status::hz(clocks.adc_hz)),
            ("CR", crate::status::hex(self.cr)),
            ("CFGR", crate::status::hex(self.cfgr)),
            ("APB1ENR", crate::status::hex(self.apb1enr)),
            ("APB2ENR", crate::status::hex(self.apb2enr)),
            ("CSR", crate::status::hex(self.csr)),
        ]
    }
}