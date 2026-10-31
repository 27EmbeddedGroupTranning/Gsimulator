//! Cortex-M3 SysTick timer (`0xE000_E010`).

use crate::bus::{extract, AccessWidth, BusError};
use crate::ctx::Ctx;
use crate::irq::exception;
use crate::time::NanoAccumulator;

const CTRL_ENABLE: u32 = 1 << 0;
const CTRL_TICKINT: u32 = 1 << 1;
const CTRL_CLKSOURCE: u32 = 1 << 2;
const CTRL_COUNTFLAG: u32 = 1 << 16;
const MAX_RELOAD: u32 = 0x00FF_FFFF;

/// Ten milliseconds expressed in the 9 MHz reference clock (`HCLK / 8`).
const CALIB_TENMS: u32 = 90_000;

/// SysTick state.
#[derive(Debug, Clone, Default)]
pub struct SysTick {
    ctrl: u32,
    load: u32,
    val: u32,
    count_flag: bool,
    acc: NanoAccumulator,
}

impl SysTick {
    pub fn new() -> Self {
        SysTick::default()
    }

    pub fn reset(&mut self) {
        *self = SysTick::new();
    }

    pub fn enabled(&self) -> bool {
        self.ctrl & CTRL_ENABLE != 0
    }

    pub fn current(&self) -> u32 {
        self.val
    }

    pub fn reload(&self) -> u32 {
        self.load
    }

    /// Advances the counter by `elapsed_ns` nanoseconds of `HCLK`.
    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if self.ctrl & CTRL_ENABLE == 0 || elapsed_ns == 0 {
            self.acc.reset();
            return;
        }
        let hclk = cx.clocks().hclk_hz;
        let clock = if self.ctrl & CTRL_CLKSOURCE != 0 {
            hclk
        } else {
            hclk / 8
        };
        let cycles = self.acc.cycles(elapsed_ns, clock);
        if cycles == 0 {
            return;
        }
        let load = self.load & MAX_RELOAD;
        let period = u64::from(load) + 1;
        let phase = u64::from(load.saturating_sub(self.val));
        let total = phase + cycles;
        let wraps = total / period;
        let new_phase = total % period;
        self.val = load - new_phase as u32;
        if wraps > 0 {
            self.count_flag = true;
            if self.ctrl & CTRL_TICKINT != 0 {
                cx.exception(exception::SYSTICK);
            }
        }
    }

    pub fn read_reg(&mut self, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let aligned = offset & !3;
        let value = match aligned {
            0x00 => {
                let value = self.ctrl | if self.count_flag { CTRL_COUNTFLAG } else { 0 };
                self.count_flag = false;
                value
            }
            0x04 => self.load & MAX_RELOAD,
            0x08 => self.val & MAX_RELOAD,
            0x0C => CALIB_TENMS,
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
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        let shift = (offset & 3) * 8;
        let lane = (value & width.mask()) << shift;
        let lane_mask = width.mask() << shift;
        match offset & !3 {
            0x00 => {
                self.ctrl = (self.ctrl & !lane_mask)
                    | (lane & (CTRL_ENABLE | CTRL_TICKINT | CTRL_CLKSOURCE));
                self.count_flag = false;
            }
            0x04 => self.load = (self.load & !lane_mask) | (lane & MAX_RELOAD),
            0x08 => {
                self.val = (self.val & !lane_mask) | (lane & MAX_RELOAD);
                self.count_flag = false;
            }
            0x0C => {}
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

impl SysTick {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, clocks: &crate::Clocks) -> Vec<(&'static str, String)> {
        let tick_hz = if self.load > 0 {
            clocks.hclk_hz / (self.load + 1)
        } else {
            0
        };
        vec![
            ("使能", if self.enabled() { "开" } else { "关" }.to_string()),
            ("重装值", format!("{}", self.load)),
            ("当前值", format!("{}", self.current())),
            (
                "计数标志",
                if self.count_flag { "1" } else { "0" }.to_string(),
            ),
            ("中断频率", format!("{tick_hz} Hz")),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::HSI_8MHZ)
    }

    #[test]
    fn counts_down_and_reloads() {
        let mut st = SysTick::new();
        // 8 MHz core clock, reload of 7 -> one wrap every 8 cycles = 1 us.
        st.write_reg(0x04, AccessWidth::Word, 7).unwrap();
        st.write_reg(0x00, AccessWidth::Word, CTRL_ENABLE | CTRL_CLKSOURCE)
            .unwrap();
        let cx = ctx();
        st.advance(&cx, 500); // 4 cycles: reload then count down to 4
        assert_eq!(st.current(), 4);
        st.advance(&cx, 375); // 3 cycles
        assert_eq!(st.current(), 1);
        st.advance(&cx, 125); // 1 cycle
        assert_eq!(st.current(), 0);
        st.advance(&cx, 125); // reload
        assert_eq!(st.current(), 7);

        // Reading CTRL reports and clears COUNTFLAG.
        assert_ne!(
            st.read_reg(0x00, AccessWidth::Word).unwrap() & CTRL_COUNTFLAG,
            0
        );
        assert_eq!(
            st.read_reg(0x00, AccessWidth::Word).unwrap() & CTRL_COUNTFLAG,
            0
        );
    }

    #[test]
    fn large_advance_wraps_many_times() {
        let mut st = SysTick::new();
        st.write_reg(0x04, AccessWidth::Word, 7).unwrap();
        st.write_reg(0x00, AccessWidth::Word, CTRL_ENABLE | CTRL_TICKINT)
            .unwrap();
        let cx = ctx();
        st.advance(&cx, 1_000_000); // 8000 cycles at 8 MHz
        let events = cx.take_events();
        assert_eq!(events.len(), 1, "one exception report per advance");
        assert_eq!(
            events[0],
            crate::ctx::Event::Exception {
                number: exception::SYSTICK
            }
        );
        assert_eq!(st.current(), 0);
    }
}
