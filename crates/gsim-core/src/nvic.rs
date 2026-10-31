//! Cortex-M3 nested vectored interrupt controller.
//!
//! The model keeps the NVIC at register level (`ISER`/`ICER`, `ISPR`/`ICPR`,
//! `IABR`, `IPR`, `STIR`) and additionally implements the arbitration rules a
//! real core applies: pre-emption by priority, tail chaining, active stacking
//! and `PRIMASK`/`BASEPRI` masking. A future core front end only has to call
//! [`Nvic::enter`] and [`Nvic::exit`] around handler execution.

use crate::bus::{extract, AccessWidth, BusError};
use crate::irq::{exception, Irq, NUM_IRQ_LINES};

const WORDS: usize = (NUM_IRQ_LINES as usize).div_ceil(32);

/// Priority bits implemented by the STM32F103 NVIC (upper nibble of each byte).
pub const IMPLEMENTED_PRIORITY_BITS: u8 = 4;
const PRIORITY_MASK: u8 = !((1u8 << (8 - IMPLEMENTED_PRIORITY_BITS)) - 1);

/// Register offsets from `0xE000_E100`.
const ISER_BASE: u32 = 0x000;
const ISER_LAST: u32 = 0x01C;
const ICER_BASE: u32 = 0x080;
const ICER_LAST: u32 = 0x09C;
const ISPR_BASE: u32 = 0x100;
const ISPR_LAST: u32 = 0x11C;
const ICPR_BASE: u32 = 0x180;
const ICPR_LAST: u32 = 0x19C;
const IABR_BASE: u32 = 0x200;
const IABR_LAST: u32 = 0x21C;
const IPR_BASE: u32 = 0x300;
const IPR_LAST: u32 = 0x3EF;
const STIR_OFFSET: u32 = 0xE00;

/// Interrupt controller state.
#[derive(Debug, Clone)]
pub struct Nvic {
    enabled: [u32; WORDS],
    pending: [u32; WORDS],
    active: [u32; WORDS],
    priority: [u8; NUM_IRQ_LINES as usize],
    system_pending: u16,
    system_active: u16,
    system_priority: [u8; 16],
    stack: Vec<u16>,
    stir: u32,
    /// Cortex-M `PRIMASK` shadow, set by the core front end.
    pub primask: bool,
    /// Cortex-M `BASEPRI` shadow, set by the core front end.
    pub basepri: u8,
    /// Cortex-M `FAULTMASK` shadow, set by the core front end.
    pub faultmask: bool,
}

impl Default for Nvic {
    fn default() -> Self {
        Nvic::new()
    }
}

impl Nvic {
    pub fn new() -> Self {
        Nvic {
            enabled: [0; WORDS],
            pending: [0; WORDS],
            active: [0; WORDS],
            priority: [0; NUM_IRQ_LINES as usize],
            system_pending: 0,
            system_active: 0,
            system_priority: [0; 16],
            stack: Vec::new(),
            stir: 0,
            primask: false,
            basepri: 0,
            faultmask: false,
        }
    }

    /// Drops every pending, active and enable bit, as a reset does.
    pub fn reset(&mut self) {
        self.enabled = [0; WORDS];
        self.pending = [0; WORDS];
        self.active = [0; WORDS];
        self.system_pending = 0;
        self.system_active = 0;
        self.stack.clear();
        self.primask = false;
        self.basepri = 0;
        self.faultmask = false;
    }

    fn slot(irq: Irq) -> Option<(usize, u32)> {
        let n = irq.number();
        if n >= NUM_IRQ_LINES {
            return None;
        }
        Some(((n / 32) as usize, 1u32 << (n % 32)))
    }

    fn system_bit(exc: u16) -> Option<u16> {
        if exc < 16 {
            Some(1u16 << exc)
        } else {
            None
        }
    }

    pub fn enable(&mut self, irq: Irq, on: bool) {
        if let Some((word, bit)) = Self::slot(irq) {
            if on {
                self.enabled[word] |= bit;
            } else {
                self.enabled[word] &= !bit;
            }
        }
    }

    pub fn is_enabled(&self, irq: Irq) -> bool {
        Self::slot(irq).is_some_and(|(word, bit)| self.enabled[word] & bit != 0)
    }

    pub fn set_pending(&mut self, irq: Irq, on: bool) {
        if let Some((word, bit)) = Self::slot(irq) {
            if on {
                self.pending[word] |= bit;
            } else {
                self.pending[word] &= !bit;
            }
        }
    }

    pub fn is_pending(&self, irq: Irq) -> bool {
        Self::slot(irq).is_some_and(|(word, bit)| self.pending[word] & bit != 0)
    }

    pub fn is_active(&self, irq: Irq) -> bool {
        Self::slot(irq).is_some_and(|(word, bit)| self.active[word] & bit != 0)
    }

    /// Sets the 8 bit priority byte, honouring the implemented priority bits.
    pub fn set_priority(&mut self, irq: Irq, priority: u8) {
        let n = irq.number();
        if n < NUM_IRQ_LINES {
            self.priority[n as usize] = priority & PRIORITY_MASK;
        }
    }

    pub fn priority(&self, irq: Irq) -> u8 {
        let n = irq.number();
        if n < NUM_IRQ_LINES {
            self.priority[n as usize]
        } else {
            0
        }
    }

    pub fn set_system_priority(&mut self, exc: u16, priority: u8) {
        if (4..16).contains(&exc) {
            self.system_priority[exc as usize] = priority & PRIORITY_MASK;
        }
    }

    pub fn system_priority(&self, exc: u16) -> u8 {
        if exc < 16 {
            self.system_priority[exc as usize]
        } else {
            0
        }
    }

    pub fn set_system_pending(&mut self, exc: u16, on: bool) {
        if let Some(bit) = Self::system_bit(exc) {
            if on {
                self.system_pending |= bit;
            } else {
                self.system_pending &= !bit;
            }
        }
    }

    pub fn is_system_pending(&self, exc: u16) -> bool {
        Self::system_bit(exc).is_some_and(|bit| self.system_pending & bit != 0)
    }

    pub fn is_system_active(&self, exc: u16) -> bool {
        Self::system_bit(exc).is_some_and(|bit| self.system_active & bit != 0)
    }

    /// Exception number of the topmost active exception, if any.
    pub fn current_exception(&self) -> Option<u16> {
        self.stack.last().copied()
    }

    /// Number of nested exceptions currently active.
    pub fn nesting_depth(&self) -> usize {
        self.stack.len()
    }

    /// Marks `exc` as active, as exception entry does.
    pub fn enter(&mut self, exc: u16) {
        if let Some(bit) = Self::system_bit(exc) {
            self.system_active |= bit;
            self.system_pending &= !bit;
        } else {
            let irq = Irq::new(exc - 16);
            if let Some((word, bit)) = Self::slot(irq) {
                self.active[word] |= bit;
                self.pending[word] &= !bit;
            }
        }
        self.stack.push(exc);
    }

    /// Marks `exc` as complete, as exception return does.
    pub fn exit(&mut self, exc: u16) {
        if let Some(bit) = Self::system_bit(exc) {
            self.system_active &= !bit;
        } else {
            let irq = Irq::new(exc - 16);
            if let Some((word, bit)) = Self::slot(irq) {
                self.active[word] &= !bit;
            }
        }
        if let Some(pos) = self.stack.iter().rposition(|&e| e == exc) {
            self.stack.remove(pos);
        }
    }

    /// Splits an 8 bit priority into its pre-emption part for `prigroup`.
    fn preempt(priority: u8, prigroup: u8) -> u16 {
        u16::from(priority) >> ((prigroup & 0x7) + 1)
    }

    fn priority_of(&self, exc: u16) -> u8 {
        if exc < 16 {
            self.system_priority[exc as usize]
        } else {
            self.priority[(exc - 16) as usize]
        }
    }

    fn preempt_of(&self, exc: u16, prigroup: u8) -> u16 {
        match exc {
            exception::NMI => 0,
            exception::HARD_FAULT => 1,
            _ => Self::preempt(self.priority_of(exc), prigroup),
        }
    }

    /// Pre-emption priority of the currently executing exception.
    ///
    /// Thread mode is `0x100`, which is lower urgency than any exception.
    pub fn current_preempt(&self, prigroup: u8) -> u16 {
        match self.current_exception() {
            None => 0x100,
            Some(exc) => self.preempt_of(exc, prigroup),
        }
    }

    /// Highest priority exception that is ready to be taken.
    ///
    /// Returns a Cortex-M exception number. `prigroup` comes from
    /// `SCB->AIRCR.PRIGROUP`.
    pub fn pending_exception(&self, prigroup: u8) -> Option<u16> {
        if self.faultmask {
            return None;
        }
        if self.system_pending & (1 << exception::NMI) != 0 {
            return Some(exception::NMI);
        }
        if self.system_pending & (1 << exception::HARD_FAULT) != 0
            && self.preempt_of(exception::HARD_FAULT, prigroup) < self.current_preempt(prigroup)
        {
            return Some(exception::HARD_FAULT);
        }
        if self.primask {
            return None;
        }

        let current = self.current_preempt(prigroup);
        let mut best: Option<(u16, u16)> = None;

        let mut consider = |exc: u16, priority: u8| {
            let preempt = Self::preempt(priority, prigroup);
            if preempt < current
                && (self.basepri == 0 || priority < self.basepri)
                && best.map_or(true, |(p, _)| preempt < p)
            {
                best = Some((preempt, exc));
            }
        };

        for exc in 9..16u16 {
            if self.system_pending & (1 << exc) != 0 && self.system_active & (1 << exc) == 0 {
                consider(exc, self.system_priority[exc as usize]);
            }
        }

        for irq_index in 0..NUM_IRQ_LINES {
            let word = (irq_index / 32) as usize;
            let bit = 1u32 << (irq_index % 32);
            if self.pending[word] & bit != 0
                && self.enabled[word] & bit != 0
                && self.active[word] & bit == 0
            {
                consider(16 + irq_index, self.priority[irq_index as usize]);
            }
        }

        best.map(|(_, exc)| exc)
    }

    /// Number of enabled and pending interrupts, used by diagnostics.
    pub fn pending_count(&self) -> u32 {
        self.pending.iter().map(|w| w.count_ones()).sum()
    }

    /// Iterates over every pending external interrupt line.
    ///
    /// A CPU front end uses this together with the enable and priority state to
    /// decide which exception to request from the core.
    pub fn pending_irqs(&self) -> impl Iterator<Item = Irq> + '_ {
        (0..NUM_IRQ_LINES).filter_map(move |line| {
            let word = (line / 32) as usize;
            let bit = 1u32 << (line % 32);
            if self.pending[word] & bit != 0 {
                Some(Irq::new(line))
            } else {
                None
            }
        })
    }

    /// Reads an NVIC register; `offset` is relative to `0xE000_E100`.
    pub fn read_reg(&self, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let aligned = offset & !3;
        let value = match aligned {
            ISER_BASE..=ISER_LAST => self.enabled[((aligned - ISER_BASE) / 4) as usize],
            ICER_BASE..=ICER_LAST => self.enabled[((aligned - ICER_BASE) / 4) as usize],
            ISPR_BASE..=ISPR_LAST => self.pending[((aligned - ISPR_BASE) / 4) as usize],
            ICPR_BASE..=ICPR_LAST => self.pending[((aligned - ICPR_BASE) / 4) as usize],
            IABR_BASE..=IABR_LAST => self.active[((aligned - IABR_BASE) / 4) as usize],
            IPR_BASE..=IPR_LAST => {
                let base = ((aligned - IPR_BASE) / 4) as usize * 4;
                let mut value = 0u32;
                for lane in 0..4 {
                    let index = base + lane;
                    if index < NUM_IRQ_LINES as usize {
                        value |= u32::from(self.priority[index]) << (lane * 8);
                    }
                }
                value
            }
            STIR_OFFSET => self.stir,
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

    /// Writes an NVIC register; `offset` is relative to `0xE000_E100`.
    pub fn write_reg(&mut self, offset: u32, width: AccessWidth, raw: u32) -> Result<(), BusError> {
        let aligned = offset & !3;
        let value = (raw & width.mask()) << ((offset & 3) * 8);
        match aligned {
            ISER_BASE..=ISER_LAST => {
                let index = ((aligned - ISER_BASE) / 4) as usize;
                self.enabled[index] |= value;
            }
            ICER_BASE..=ICER_LAST => {
                let index = ((aligned - ICER_BASE) / 4) as usize;
                self.enabled[index] &= !value;
            }
            ISPR_BASE..=ISPR_LAST => {
                let index = ((aligned - ISPR_BASE) / 4) as usize;
                self.pending[index] |= value;
            }
            ICPR_BASE..=ICPR_LAST => {
                let index = ((aligned - ICPR_BASE) / 4) as usize;
                self.pending[index] &= !value;
            }
            IABR_BASE..=IABR_LAST => {}
            IPR_BASE..=IPR_LAST => {
                let first_byte = ((aligned - IPR_BASE) + (offset & 3)) as usize;
                for lane_index in 0..width.bytes() {
                    let index = first_byte + lane_index as usize;
                    if index < NUM_IRQ_LINES as usize {
                        let byte = ((raw >> (lane_index * 8)) & 0xFF) as u8;
                        self.priority[index] = byte & PRIORITY_MASK;
                    }
                }
            }
            STIR_OFFSET => {
                let irq = value & 0x1FF;
                self.stir = irq;
                if irq < u32::from(NUM_IRQ_LINES) {
                    self.set_pending(Irq::new(irq as u16), true);
                }
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

impl Nvic {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &crate::Clocks) -> Vec<(&'static str, String)> {
        let active = self
            .active
            .iter()
            .map(|word| word.count_ones())
            .sum::<u32>();
        let enabled = self
            .enabled
            .iter()
            .map(|word| word.count_ones())
            .sum::<u32>();
        vec![
            ("已使能中断", format!("{enabled}")),
            ("挂起中断", format!("{}", self.pending_count())),
            ("活动中断", format!("{active}")),
            (
                "当前异常",
                match self.current_exception() {
                    Some(number) => format!("#{number}"),
                    None => "无".to_string(),
                },
            ),
            ("嵌套深度", format!("{}", self.nesting_depth())),
            (
                "PRIMASK / BASEPRI",
                format!(
                    "{} / 0x{:02X}",
                    if self.primask { 1 } else { 0 },
                    self.basepri
                ),
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::irq::lines;

    #[test]
    fn pending_line_is_taken_once_enabled() {
        let mut nvic = Nvic::new();
        nvic.set_pending(lines::USART1, true);
        assert_eq!(nvic.pending_exception(0), None);
        nvic.enable(lines::USART1, true);
        assert_eq!(nvic.pending_exception(0), Some(lines::USART1.exception()));
    }

    #[test]
    fn lower_priority_value_wins() {
        let mut nvic = Nvic::new();
        nvic.enable(lines::TIM2, true);
        nvic.enable(lines::USART1, true);
        nvic.set_priority(lines::TIM2, 0x80);
        nvic.set_priority(lines::USART1, 0x10);
        nvic.set_pending(lines::TIM2, true);
        nvic.set_pending(lines::USART1, true);
        assert_eq!(nvic.pending_exception(0), Some(lines::USART1.exception()));
    }

    #[test]
    fn equal_priority_uses_lower_exception_number() {
        let mut nvic = Nvic::new();
        nvic.enable(lines::USART1, true);
        nvic.enable(lines::TIM2, true);
        nvic.set_priority(lines::USART1, 0x00);
        nvic.set_priority(lines::TIM2, 0x00);
        nvic.set_pending(lines::TIM2, true);
        nvic.set_pending(lines::USART1, true);
        // TIM2 has IRQ 28, USART1 has IRQ 37.
        assert_eq!(nvic.pending_exception(0), Some(lines::TIM2.exception()));
    }

    #[test]
    fn running_handler_blocks_equal_priority_but_not_higher() {
        let mut nvic = Nvic::new();
        nvic.enable(lines::TIM2, true);
        nvic.enable(lines::USART1, true);
        nvic.set_priority(lines::TIM2, 0x40);
        nvic.set_priority(lines::USART1, 0x40);
        let tim2 = lines::TIM2.exception();
        nvic.enter(tim2);

        nvic.set_pending(lines::TIM2, true);
        nvic.set_pending(lines::USART1, true);
        assert_eq!(nvic.pending_exception(0), None);

        nvic.set_priority(lines::USART1, 0x00);
        assert_eq!(nvic.pending_exception(0), Some(lines::USART1.exception()));

        nvic.exit(tim2);
        assert_eq!(nvic.current_exception(), None);
        assert_eq!(nvic.pending_exception(0), Some(lines::USART1.exception()));
    }

    #[test]
    fn primask_and_basepri_mask_exceptions() {
        let mut nvic = Nvic::new();
        nvic.enable(lines::TIM3, true);
        nvic.set_priority(lines::TIM3, 0x20);
        nvic.set_pending(lines::TIM3, true);

        nvic.primask = true;
        assert_eq!(nvic.pending_exception(0), None);
        nvic.primask = false;
        assert!(nvic.pending_exception(0).is_some());

        nvic.basepri = 0x10;
        assert_eq!(
            nvic.pending_exception(0),
            None,
            "priority 0x20 is masked by BASEPRI 0x10"
        );
        nvic.basepri = 0x40;
        assert!(nvic.pending_exception(0).is_some());
    }

    #[test]
    fn systick_exception_is_arbitrated_with_irqs() {
        let mut nvic = Nvic::new();
        nvic.enable(lines::USART1, true);
        nvic.set_priority(lines::USART1, 0xF0);
        nvic.set_system_priority(exception::SYSTICK, 0x10);
        nvic.set_pending(lines::USART1, true);
        nvic.set_system_pending(exception::SYSTICK, true);
        assert_eq!(
            nvic.pending_exception(0),
            Some(exception::SYSTICK),
            "SysTick has the higher priority"
        );
    }

    #[test]
    fn registers_behave_like_the_hardware() {
        let mut nvic = Nvic::new();
        // ISER0 write of bit 5 enables IRQ 5.
        nvic.write_reg(0x00, AccessWidth::Word, 1 << 5).unwrap();
        assert!(nvic.is_enabled(Irq::new(5)));
        assert_eq!(nvic.read_reg(0x00, AccessWidth::Word).unwrap(), 1 << 5);

        // ICER0 clears it again.
        nvic.write_reg(0x80, AccessWidth::Word, 1 << 5).unwrap();
        assert!(!nvic.is_enabled(Irq::new(5)));

        // IPR byte access for IRQ 5 (byte offset 5 within IPR).
        nvic.write_reg(0x305, AccessWidth::Byte, 0xAB).unwrap();
        assert_eq!(nvic.priority(Irq::new(5)), 0xA0);
        assert_eq!(nvic.read_reg(0x305, AccessWidth::Byte).unwrap(), 0xA0);

        // STIR pends an interrupt.
        nvic.write_reg(0xE00, AccessWidth::Word, 7).unwrap();
        assert!(nvic.is_pending(Irq::new(7)));
    }
}
