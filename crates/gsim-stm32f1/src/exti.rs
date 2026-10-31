//! External interrupt/event controller (EXTI).
//!
//! The SoC feeds the 16 line levels into [`Exti::update`] once per simulation
//! step; the block latches edges into `PR` and drives the matching NVIC lines.

use super::map::exti as reg;
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::lines::EXTI as EXTI_IRQ;

/// Number of EXTI lines.
pub const LINE_COUNT: usize = 16;

/// External interrupt controller state.
#[derive(Debug, Clone)]
pub struct Exti {
    imr: u32,
    emr: u32,
    rtsr: u32,
    ftsr: u32,
    swier: u32,
    pr: u32,
    previous: [bool; LINE_COUNT],
    irq_state: [bool; LINE_COUNT],
}

impl Default for Exti {
    fn default() -> Self {
        Exti::new()
    }
}

impl Exti {
    pub fn new() -> Self {
        Exti {
            imr: 0,
            emr: 0,
            rtsr: 0,
            ftsr: 0,
            swier: 0,
            pr: 0,
            previous: [false; LINE_COUNT],
            irq_state: [false; LINE_COUNT],
        }
    }

    pub fn reset(&mut self) {
        *self = Exti::new();
    }

    /// Pending register value.
    pub fn pending(&self) -> u32 {
        self.pr
    }

    /// Records the current line levels without latching anything.
    ///
    /// The SoC calls this whenever the EXTI configuration changes so that a
    /// level which was already present before the line was enabled is not
    /// mistaken for an edge.
    pub fn prime(&mut self, levels: &[bool; LINE_COUNT]) {
        self.previous = *levels;
    }

    /// Latches edges detected on `levels` and refreshes the NVIC lines.
    pub fn update(&mut self, cx: &Ctx, levels: &[bool; LINE_COUNT]) {
        for (line, level) in levels.iter().enumerate() {
            let bit = 1u32 << line;
            if self.imr & bit != 0 || self.emr & bit != 0 {
                if *level && !self.previous[line] && self.rtsr & bit != 0 {
                    self.pr |= bit;
                }
                if !*level && self.previous[line] && self.ftsr & bit != 0 {
                    self.pr |= bit;
                }
            }
            self.previous[line] = *level;
        }
        self.refresh_irq(cx);
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        for (line, state) in self.irq_state.iter_mut().enumerate() {
            let bit = 1u32 << line;
            let high = self.pr & bit != 0 && self.imr & bit != 0;
            if high != *state {
                *state = high;
                cx.irq(EXTI_IRQ[line], high);
            }
        }
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::IMR => self.imr,
            reg::EMR => self.emr,
            reg::RTSR => self.rtsr,
            reg::FTSR => self.ftsr,
            reg::SWIER => self.swier,
            reg::PR => self.pr,
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
        const MASK: u32 = 0x0000_FFFF;
        match offset & !3 {
            reg::IMR => self.imr = merge(self.imr, offset, width, value) & MASK,
            reg::EMR => self.emr = merge(self.emr, offset, width, value) & MASK,
            reg::RTSR => self.rtsr = merge(self.rtsr, offset, width, value) & MASK,
            reg::FTSR => self.ftsr = merge(self.ftsr, offset, width, value) & MASK,
            reg::SWIER => {
                let set = (merge(self.swier, offset, width, value)) & MASK;
                self.swier |= set;
                self.pr |= set;
            }
            reg::PR => {
                let clear = merge(0, offset, width, value) & MASK;
                self.pr &= !clear;
            }
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

    pub fn advance(&mut self, _cx: &Ctx, _elapsed_ns: u64) {}
}

crate::impl_peripheral!(Exti, "EXTI");

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;
    use gsim_core::irq::lines;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::default())
    }

    #[test]
    fn rising_edge_latches_and_raises_irq() {
        let cx = ctx();
        let mut exti = Exti::new();
        exti.write_reg(&cx, reg::IMR, AccessWidth::Word, 1).unwrap();
        exti.write_reg(&cx, reg::RTSR, AccessWidth::Word, 1)
            .unwrap();
        cx.take_events();

        let mut levels = [false; LINE_COUNT];
        exti.update(&cx, &levels);
        levels[0] = true;
        exti.update(&cx, &levels);

        assert_eq!(exti.pending() & 1, 1);
        let events = cx.take_events();
        assert_eq!(
            events,
            vec![gsim_core::Event::Irq {
                irq: lines::EXTI0,
                high: true
            }]
        );
    }

    #[test]
    fn irq_line_releases_when_pending_is_cleared() {
        let cx = ctx();
        let mut exti = Exti::new();
        exti.write_reg(&cx, reg::IMR, AccessWidth::Word, 1 << 3)
            .unwrap();
        exti.write_reg(&cx, reg::FTSR, AccessWidth::Word, 1 << 3)
            .unwrap();
        let mut levels = [true; LINE_COUNT];
        exti.update(&cx, &levels);
        levels[3] = false;
        exti.update(&cx, &levels);
        cx.take_events();

        exti.write_reg(&cx, reg::PR, AccessWidth::Word, 1 << 3)
            .unwrap();
        assert_eq!(exti.pending() & (1 << 3), 0);
        assert_eq!(
            cx.take_events(),
            vec![gsim_core::Event::Irq {
                irq: lines::EXTI3,
                high: false
            }]
        );
    }

    #[test]
    fn software_interrupt_sets_pending_bit() {
        let cx = ctx();
        let mut exti = Exti::new();
        exti.write_reg(&cx, reg::IMR, AccessWidth::Word, 1 << 9)
            .unwrap();
        exti.write_reg(&cx, reg::SWIER, AccessWidth::Word, 1 << 9)
            .unwrap();
        assert_eq!(exti.pending() & (1 << 9), 1 << 9);
        let events = cx.take_events();
        assert_eq!(
            events[0],
            gsim_core::Event::Irq {
                irq: lines::EXTI9_5,
                high: true
            }
        );
    }

    #[test]
    fn masked_line_does_not_latch() {
        let cx = ctx();
        let mut exti = Exti::new();
        exti.write_reg(&cx, reg::RTSR, AccessWidth::Word, 1)
            .unwrap();
        let mut levels = [false; LINE_COUNT];
        exti.update(&cx, &levels);
        levels[0] = true;
        exti.update(&cx, &levels);
        assert_eq!(exti.pending(), 0, "IMR is clear so nothing latches");
    }
}

impl Exti {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("中断屏蔽 IMR", crate::status::hex(self.imr)),
            ("事件屏蔽 EMR", crate::status::hex(self.emr)),
            ("上升沿 RTSR", crate::status::hex(self.rtsr)),
            ("下降沿 FTSR", crate::status::hex(self.ftsr)),
            ("挂起 PR", crate::status::hex(self.pr)),
        ]
    }
}