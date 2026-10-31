//! Advanced-control and general purpose timers (TIM1..TIM4).
//!
//! The model counts at the real prescaled clock rate, generates update events
//! on overflow/underflow (with the repetition counter of the advanced timer),
//! latches compare matches, supports the four output-compare/PWM channels and
//! input capture from the channel pins. Preload registers (`ARPE`, `OCxPE`)
//! are applied immediately, which keeps the PWM duty observable in the same
//! period the firmware wrote it.

use super::map::tim as reg;
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::Irq;
use gsim_core::time::NanoAccumulator;

/// `CR1` bits.
pub mod cr1 {
    pub const CEN: u32 = 1 << 0;
    pub const UDIS: u32 = 1 << 1;
    pub const URS: u32 = 1 << 2;
    pub const OPM: u32 = 1 << 3;
    pub const DIR: u32 = 1 << 4;
    pub const CMS: u32 = 0b11 << 5;
    pub const ARPE: u32 = 1 << 7;
    pub const CKD: u32 = 0b11 << 8;
}

/// `DIER` bits.
pub mod dier {
    pub const UIE: u32 = 1 << 0;
    pub const CC1IE: u32 = 1 << 1;
    pub const CC2IE: u32 = 1 << 2;
    pub const CC3IE: u32 = 1 << 3;
    pub const CC4IE: u32 = 1 << 4;
    pub const COMIE: u32 = 1 << 5;
    pub const TIE: u32 = 1 << 6;
    pub const BIE: u32 = 1 << 7;
    pub const UDE: u32 = 1 << 8;
    pub const CC1DE: u32 = 1 << 9;
    pub const CC2DE: u32 = 1 << 10;
    pub const CC3DE: u32 = 1 << 11;
    pub const CC4DE: u32 = 1 << 12;
}

/// `SR` bits.
pub mod sr {
    pub const UIF: u32 = 1 << 0;
    pub const CC1IF: u32 = 1 << 1;
    pub const CC2IF: u32 = 1 << 2;
    pub const CC3IF: u32 = 1 << 3;
    pub const CC4IF: u32 = 1 << 4;
    pub const COMIF: u32 = 1 << 5;
    pub const TIF: u32 = 1 << 6;
    pub const BIF: u32 = 1 << 7;
    pub const CC1OF: u32 = 1 << 9;
}

/// `EGR` bits.
pub mod egr {
    pub const UG: u32 = 1 << 0;
    pub const CC1G: u32 = 1 << 1;
    pub const CC2G: u32 = 1 << 2;
    pub const CC3G: u32 = 1 << 3;
    pub const CC4G: u32 = 1 << 4;
    pub const COMG: u32 = 1 << 5;
    pub const TG: u32 = 1 << 6;
    pub const BG: u32 = 1 << 7;
}

/// `CCER` bit that enables an output channel.
pub fn cc_enable_bit(channel: usize) -> u32 {
    1 << (channel * 4)
}

/// `CCER` bit that inverts an output channel polarity.
pub fn cc_polarity_bit(channel: usize) -> u32 {
    1 << (channel * 4 + 1)
}

/// `CCxS` selection bits inside `CCMR1`/`CCMR2`.
fn ccmr_channel_shift(channel: usize) -> u32 {
    if channel % 2 == 0 {
        0
    } else {
        8
    }
}

fn ccmr_register_offset(channel: usize) -> u32 {
    if channel < 2 {
        reg::CCMR1
    } else {
        reg::CCMR2
    }
}

const MAX_FINE_TICKS: u64 = 100_000;

/// One timer instance.
#[derive(Debug, Clone)]
pub struct Tim {
    index: u8,
    update_irq: Irq,
    cc_irq: Irq,
    advanced: bool,
    cr1: u32,
    cr2: u32,
    smcr: u32,
    dier: u32,
    sr: u32,
    ccmr1: u32,
    ccmr2: u32,
    ccer: u32,
    cnt: u32,
    psc: u32,
    arr: u32,
    rcr: u32,
    ccr: [u32; 4],
    bdtr: u32,
    dcr: u32,
    dmar: u32,
    prescaler_counter: u32,
    repetition: u32,
    counting_down: bool,
    acc: NanoAccumulator,
    input_levels: [bool; 4],
    input_previous: [bool; 4],
    update_irq_high: bool,
    cc_irq_high: bool,
    update_dma_request: bool,
    cc_dma_request: [bool; 4],
}

impl Tim {
    /// Creates a timer. `advanced` enables `RCR`, `BDTR` and the break input.
    pub fn new(index: u8, update_irq: Irq, cc_irq: Irq, advanced: bool) -> Self {
        Tim {
            index,
            update_irq,
            cc_irq,
            advanced,
            cr1: 0,
            cr2: 0,
            smcr: 0,
            dier: 0,
            sr: 0,
            ccmr1: 0,
            ccmr2: 0,
            ccer: 0,
            cnt: 0,
            psc: 0,
            arr: 0xFFFF,
            rcr: 0,
            ccr: [0; 4],
            bdtr: 0,
            dcr: 0,
            dmar: 0,
            prescaler_counter: 0,
            repetition: 0,
            counting_down: false,
            acc: NanoAccumulator::new(),
            input_levels: [false; 4],
            input_previous: [false; 4],
            update_irq_high: false,
            cc_irq_high: false,
            update_dma_request: false,
            cc_dma_request: [false; 4],
        }
    }

    pub fn reset(&mut self) {
        let (index, update_irq, cc_irq, advanced) =
            (self.index, self.update_irq, self.cc_irq, self.advanced);
        *self = Tim::new(index, update_irq, cc_irq, advanced);
    }

    /// Timer index (0 = TIM1).
    pub fn index(&self) -> u8 {
        self.index
    }

    /// Current counter value.
    pub fn counter(&self) -> u32 {
        self.cnt
    }

    /// True while the counter is enabled.
    pub fn running(&self) -> bool {
        self.cr1 & cr1::CEN != 0
    }

    /// Drives a channel input pin, used for input capture.
    pub fn set_channel_input(&mut self, channel: usize, high: bool) {
        if channel < 4 {
            self.input_levels[channel] = high;
        }
    }

    fn clock_hz(&self, cx: &Ctx) -> u32 {
        if self.index == 0 {
            cx.clocks().tim1_hz
        } else {
            cx.clocks().tim2_hz
        }
    }

    fn channel_is_input(&self, channel: usize) -> bool {
        let ccmr = if channel < 2 { self.ccmr1 } else { self.ccmr2 };
        field(ccmr, ccmr_channel_shift(channel), 2) != 0
    }

    fn output_mode(&self, channel: usize) -> u32 {
        let ccmr = if channel < 2 { self.ccmr1 } else { self.ccmr2 };
        field(ccmr, ccmr_channel_shift(channel) + 4, 3)
    }

    /// Effective level of a compare output, honouring the output mode and
    /// polarity. Returns `None` when the channel is not an output.
    pub fn output_level(&self, channel: usize) -> Option<bool> {
        if channel >= 4 || self.channel_is_input(channel) {
            return None;
        }
        let active = match self.output_mode(channel) {
            4 => false,
            5 => true,
            6 => {
                if self.cr1 & cr1::DIR != 0 {
                    self.cnt > self.ccr[channel]
                } else {
                    self.cnt < self.ccr[channel]
                }
            }
            7 => {
                if self.cr1 & cr1::DIR != 0 {
                    self.cnt <= self.ccr[channel]
                } else {
                    self.cnt >= self.ccr[channel]
                }
            }
            _ => self.cnt == self.ccr[channel],
        };
        let inverted = self.ccer & cc_polarity_bit(channel) != 0;
        Some(active != inverted)
    }

    fn update_event(&mut self, cx: &Ctx, from_software: bool) {
        if self.cr1 & cr1::UDIS != 0 && !from_software {
            return;
        }
        let counter_only = self.cr1 & cr1::URS != 0;
        if !(from_software && counter_only) {
            self.sr |= sr::UIF;
        }
        if self.advanced {
            if self.repetition == 0 {
                self.repetition = self.rcr & 0xFF;
            } else {
                self.repetition -= 1;
            }
        }
        if self.dier & dier::UDE != 0 {
            self.update_dma_request = true;
        }
        if self.cr1 & cr1::OPM != 0 {
            self.cr1 &= !cr1::CEN;
        }
        self.refresh_irq(cx);
    }

    fn compare_match(&mut self, cx: &Ctx, channel: usize) {
        self.sr |= sr::CC1IF << channel;
        if channel < 4 && self.dier & (dier::CC1DE << channel) != 0 {
            self.cc_dma_request[channel] = true;
        }
        self.refresh_irq(cx);
    }

    fn capture(&mut self, cx: &Ctx, channel: usize) {
        self.ccr[channel] = self.cnt;
        self.sr |= sr::CC1IF << channel;
        self.refresh_irq(cx);
    }

    fn sample_inputs(&mut self, cx: &Ctx) {
        for channel in 0..4 {
            let level = self.input_levels[channel];
            let previous = self.input_previous[channel];
            if level == previous {
                continue;
            }
            self.input_previous[channel] = level;
            if !self.channel_is_input(channel) {
                continue;
            }
            let rising_selected = self.ccer & cc_polarity_bit(channel) == 0;
            let edge = if rising_selected { level } else { !level };
            if edge {
                self.capture(cx, channel);
            }
        }
    }

    fn tick(&mut self, cx: &Ctx) {
        let arr = self.arr & 0xFFFF;
        let center = self.cr1 & cr1::CMS != 0;
        let down = self.cr1 & cr1::DIR != 0;

        if center {
            if self.counting_down {
                if self.cnt <= 1 {
                    self.counting_down = false;
                    self.update_event(cx, false);
                } else {
                    self.cnt -= 1;
                }
            } else if self.cnt + 1 >= arr {
                self.counting_down = true;
            } else {
                self.cnt += 1;
            }
        } else if down {
            if self.cnt == 0 {
                self.cnt = arr;
                self.update_event(cx, false);
            } else {
                self.cnt -= 1;
            }
        } else if self.cnt + 1 > arr {
            self.cnt = 0;
            self.update_event(cx, false);
        } else {
            self.cnt += 1;
        }

        let count = self.cnt;
        for channel in 0..4 {
            if self.channel_is_input(channel) {
                continue;
            }
            if count == self.ccr[channel] {
                self.compare_match(cx, channel);
            }
        }
        self.sample_inputs(cx);
    }

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if self.cr1 & cr1::CEN == 0 {
            self.acc.reset();
            return;
        }
        let clock = self.clock_hz(cx);
        let cycles = self.acc.cycles(elapsed_ns, clock);
        if cycles == 0 {
            return;
        }
        let divisor = u64::from(self.psc) + 1;
        let total = u64::from(self.prescaler_counter) + cycles;
        let ticks = total / divisor;
        self.prescaler_counter = (total % divisor) as u32;
        if ticks == 0 {
            return;
        }
        if ticks <= MAX_FINE_TICKS {
            for _ in 0..ticks {
                if self.cr1 & cr1::CEN == 0 {
                    break;
                }
                self.tick(cx);
            }
        } else {
            self.coarse_advance(cx, ticks);
        }
        self.refresh_irq(cx);
    }

    /// Handles very large steps without iterating every clock, keeping the
    /// counter and the update event correct while collapsing intermediate
    /// compare matches.
    fn coarse_advance(&mut self, cx: &Ctx, ticks: u64) {
        if self.cr1 & cr1::CMS == 0 && self.cr1 & cr1::DIR == 0 {
            let period = u64::from(self.arr & 0xFFFF) + 1;
            let total = u64::from(self.cnt) + ticks;
            self.cnt = (total % period) as u32;
            if total / period > 0 {
                self.update_event(cx, false);
            }
        } else {
            for _ in 0..MAX_FINE_TICKS {
                self.tick(cx);
            }
        }
        cx.note(
            "TIM",
            "coarse advance collapsed events",
            u32::from(self.index),
        );
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let update_flags = self.sr & (sr::UIF | sr::TIF | sr::BIF | sr::COMIF);
        let update_enables = self.dier & (dier::UIE | dier::TIE | dier::BIE | dier::COMIE);
        if self.update_irq == self.cc_irq {
            let cc_flags = self.sr & 0x1E;
            let cc_enables = self.dier & 0x1E;
            let high = (update_flags & update_enables) != 0 || (cc_flags & cc_enables) != 0;
            if high != self.update_irq_high {
                self.update_irq_high = high;
                self.cc_irq_high = high;
                cx.irq(self.update_irq, high);
            }
            return;
        }
        let update_high = (update_flags & update_enables) != 0;
        if update_high != self.update_irq_high {
            self.update_irq_high = update_high;
            cx.irq(self.update_irq, update_high);
        }
        let cc_high = (self.sr & 0x1E) & (self.dier & 0x1E) != 0;
        if cc_high != self.cc_irq_high {
            self.cc_irq_high = cc_high;
            cx.irq(self.cc_irq, cc_high);
        }
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::CR1 => self.cr1,
            reg::CR2 => self.cr2,
            reg::SMCR => self.smcr,
            reg::DIER => self.dier,
            reg::SR => self.sr,
            reg::EGR => 0,
            reg::CCMR1 => self.ccmr1,
            reg::CCMR2 => self.ccmr2,
            reg::CCER => self.ccer,
            reg::CNT => self.cnt,
            reg::PSC => self.psc,
            reg::ARR => self.arr,
            reg::RCR => self.rcr,
            reg::CCR1 => self.ccr[0],
            reg::CCR2 => self.ccr[1],
            reg::CCR3 => self.ccr[2],
            reg::CCR4 => self.ccr[3],
            reg::BDTR => self.bdtr,
            reg::DCR => self.dcr,
            reg::DMAR => self.dmar,
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
            reg::CR1 => self.cr1 = merge(self.cr1, offset, width, value) & 0x03FF,
            reg::CR2 => self.cr2 = merge(self.cr2, offset, width, value) & 0x00FF,
            reg::SMCR => self.smcr = merge(self.smcr, offset, width, value) & 0x7777,
            reg::DIER => self.dier = merge(self.dier, offset, width, value) & 0x5FFF,
            reg::SR => self.sr &= !(value & 0x02FF),
            reg::EGR => self.write_egr(cx, value & 0x00FF),
            reg::CCMR1 => self.ccmr1 = merge(self.ccmr1, offset, width, value) & 0xFFFF,
            reg::CCMR2 => self.ccmr2 = merge(self.ccmr2, offset, width, value) & 0xFFFF,
            reg::CCER => self.ccer = merge(self.ccer, offset, width, value) & 0x3333,
            reg::CNT => self.cnt = merge(self.cnt, offset, width, value) & 0xFFFF,
            reg::PSC => self.psc = merge(self.psc, offset, width, value) & 0xFFFF,
            reg::ARR => self.arr = merge(self.arr, offset, width, value) & 0xFFFF,
            reg::RCR => self.rcr = merge(self.rcr, offset, width, value) & 0x00FF,
            reg::CCR1 => self.ccr[0] = merge(self.ccr[0], offset, width, value) & 0xFFFF,
            reg::CCR2 => self.ccr[1] = merge(self.ccr[1], offset, width, value) & 0xFFFF,
            reg::CCR3 => self.ccr[2] = merge(self.ccr[2], offset, width, value) & 0xFFFF,
            reg::CCR4 => self.ccr[3] = merge(self.ccr[3], offset, width, value) & 0xFFFF,
            reg::BDTR => self.bdtr = merge(self.bdtr, offset, width, value) & 0xFFFF,
            reg::DCR => self.dcr = merge(self.dcr, offset, width, value) & 0x1F1F,
            reg::DMAR => self.dmar = merge(self.dmar, offset, width, value) & 0xFFFF,
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: true,
                })
            }
        }
        self.refresh_irq(cx);
        Ok(())
    }

    fn write_egr(&mut self, cx: &Ctx, value: u32) {
        if value & egr::UG != 0 {
            self.cnt = 0;
            self.prescaler_counter = 0;
            self.counting_down = false;
            self.update_event(cx, true);
        }
        for channel in 0..4 {
            if value & (egr::CC1G << channel) != 0 {
                if self.channel_is_input(channel) {
                    self.capture(cx, channel);
                } else {
                    self.compare_match(cx, channel);
                }
            }
        }
        if value & egr::TG != 0 {
            self.sr |= sr::TIF;
        }
        if value & egr::BG != 0 {
            self.sr |= sr::BIF;
        }
        if value & egr::COMG != 0 {
            self.sr |= sr::COMIF;
        }
        self.refresh_irq(cx);
    }

    /// True when the update DMA request line is asserted.
    pub fn update_dma_request(&self) -> bool {
        self.update_dma_request
    }

    /// True when the compare DMA request line of `channel` is asserted.
    pub fn cc_dma_request(&self, channel: usize) -> bool {
        self.cc_dma_request.get(channel).copied().unwrap_or(false)
    }

    /// Clears a DMA request after the controller acknowledged it.
    pub fn clear_update_dma_request(&mut self) {
        self.update_dma_request = false;
    }

    /// Clears a compare DMA request after the controller acknowledged it.
    pub fn clear_cc_dma_request(&mut self, channel: usize) {
        if channel < 4 {
            self.cc_dma_request[channel] = false;
        }
    }

    /// Channel capture/compare register value.
    pub fn ccr(&self, channel: usize) -> u32 {
        self.ccr.get(channel).copied().unwrap_or(0)
    }

    /// Offset of a `CCR` register, used by tests and by the demo.
    pub fn ccr_offset(channel: usize) -> u32 {
        reg::CCR1 + 4 * channel as u32
    }

    /// Offset of the `CCMR` register that holds `channel`'s configuration.
    pub fn ccmr_offset(channel: usize) -> u32 {
        ccmr_register_offset(channel)
    }
}

crate::impl_peripheral!(Tim, "TIM");

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;
    use gsim_core::irq::lines;

    fn ctx_8mhz() -> Ctx {
        Ctx::new(0, Clocks::HSI_8MHZ)
    }

    fn timer() -> Tim {
        Tim::new(1, lines::TIM3, lines::TIM3, false)
    }

    /// Nanoseconds for `n` cycles of the 8 MHz clock used by these tests.
    fn cycles(n: u64) -> u64 {
        n * 125
    }

    #[test]
    fn counter_overflows_at_arr_and_sets_uif() {
        let cx = ctx_8mhz();
        let mut tim = timer();
        tim.write_reg(&cx, reg::PSC, AccessWidth::Word, 0).unwrap();
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 9).unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN)
            .unwrap();
        tim.advance(&cx, cycles(10));
        assert_eq!(tim.counter(), 0);
        assert_eq!(
            tim.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::UIF,
            sr::UIF
        );
    }

    #[test]
    fn prescaler_divides_the_counter_clock() {
        let cx = ctx_8mhz();
        let mut tim = timer();
        tim.write_reg(&cx, reg::PSC, AccessWidth::Word, 7).unwrap();
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 999)
            .unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN)
            .unwrap();
        // Prescaler /8 -> 1 MHz counter clock, so 100 us is 100 ticks.
        tim.advance(&cx, 100_000);
        assert_eq!(tim.counter(), 100);
    }

    #[test]
    fn pwm_duty_is_visible_through_output_level() {
        let cx = ctx_8mhz();
        let mut tim = timer();
        tim.write_reg(&cx, reg::PSC, AccessWidth::Word, 0).unwrap();
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 9).unwrap();
        // Channel 1: PWM mode 1 (OC1M = 110) inside CCMR1 bits 6:4.
        tim.write_reg(&cx, reg::CCMR1, AccessWidth::Word, 0b110 << 4)
            .unwrap();
        tim.write_reg(&cx, reg::CCER, AccessWidth::Word, cc_enable_bit(0))
            .unwrap();
        tim.write_reg(&cx, reg::CCR1, AccessWidth::Word, 4).unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN)
            .unwrap();

        assert_eq!(tim.output_level(0), Some(true), "CNT 0 < CCR 4");
        tim.advance(&cx, cycles(4));
        assert_eq!(tim.counter(), 4);
        assert_eq!(tim.output_level(0), Some(false), "CNT 4 >= CCR 4");
    }

    #[test]
    fn compare_match_sets_cc_flag_and_irq() {
        let cx = ctx_8mhz();
        let mut tim = Tim::new(1, lines::TIM3, lines::TIM3, false);
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 99).unwrap();
        tim.write_reg(&cx, reg::CCR1, AccessWidth::Word, 5).unwrap();
        tim.write_reg(&cx, reg::DIER, AccessWidth::Word, dier::CC1IE)
            .unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN)
            .unwrap();
        cx.take_events();
        tim.advance(&cx, cycles(6));
        assert_ne!(
            tim.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::CC1IF,
            0
        );
        assert!(cx.take_events().iter().any(
            |e| matches!(e, gsim_core::Event::Irq { irq, high: true } if *irq == lines::TIM3)
        ));
    }

    #[test]
    fn input_capture_latches_the_counter() {
        let cx = ctx_8mhz();
        let mut tim = timer();
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 0xFFFF)
            .unwrap();
        // Channel 1 as input, capture on rising edge (CC1S = 01).
        tim.write_reg(&cx, reg::CCMR1, AccessWidth::Word, 0b01)
            .unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN)
            .unwrap();
        tim.advance(&cx, cycles(20));
        tim.set_channel_input(0, true);
        tim.advance(&cx, cycles(1));
        assert_eq!(tim.ccr(0), 21);
    }

    #[test]
    fn one_pulse_mode_stops_after_update() {
        let cx = ctx_8mhz();
        let mut tim = timer();
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 4).unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN | cr1::OPM)
            .unwrap();
        tim.advance(&cx, cycles(6));
        assert!(!tim.running(), "OPM clears CEN on the update event");
    }

    #[test]
    fn large_step_keeps_counter_consistent() {
        let cx = ctx_8mhz();
        let mut tim = timer();
        tim.write_reg(&cx, reg::ARR, AccessWidth::Word, 999)
            .unwrap();
        tim.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::CEN)
            .unwrap();
        // 200 ms at 1 kHz counter clock = 200 periods.
        tim.advance(&cx, 200_000_000);
        assert_eq!(tim.counter(), 0);
        assert_ne!(
            tim.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::UIF,
            0
        );
    }
}

impl Tim {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        let clock = if self.index == 0 {
            clocks.tim1_hz
        } else {
            clocks.tim2_hz
        };
        let tick = clock / (self.psc + 1);
        let mut channels = 0;
        for channel in 0..4 {
            if self.ccer & (1 << (channel * 4)) != 0 {
                channels += 1;
            }
        }
        vec![
            ("使能", crate::status::on_off(self.cr1 & 1 != 0).to_string()),
            (
                "计数方向",
                if self.cr1 & (1 << 4) != 0 { "向下" } else { "向上" }.to_string(),
            ),
            ("计数频率", crate::status::hz(tick)),
            ("PSC", format!("{}", self.psc)),
            ("ARR", format!("{}", self.arr)),
            ("CNT", format!("{}", self.cnt)),
            (
                "比较通道",
                format!(
                    "{channels} 个输出（CCR1..4 = {} {} {} {}）",
                    self.ccr[0], self.ccr[1], self.ccr[2], self.ccr[3]
                ),
            ),
        ]
    }
}