//! General purpose I/O ports (GPIOA..GPIOE).
//!
//! The model reproduces the eight configurable pins per port, the atomic
//! `BSRR`/`BRR` write registers and the configuration lock (`LCKR`). Reading
//! `IDR` returns what a pin would actually sample: the output latch for pins
//! configured as output, the pull resistor value for pull-up/pull-down inputs
//! and the externally driven level for floating inputs.

use super::map::gpio as reg;
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::{Ctx, Event};

/// Number of GPIO ports modelled (A..E).
pub const PORT_COUNT: usize = 5;
/// Port letters, index compatible with [`PORT_COUNT`].
pub const PORT_NAMES: [&str; PORT_COUNT] = ["A", "B", "C", "D", "E"];

const LCKK: u32 = 1 << 16;

/// One GPIO port.
#[derive(Debug, Clone)]
pub struct GpioPort {
    index: u8,
    crl: u32,
    crh: u32,
    odr: u32,
    lckr: u32,
    locked: u32,
    lock_stage: u8,
    external: u16,
    driven: u16,
}

impl GpioPort {
    pub fn new(index: u8) -> Self {
        GpioPort {
            index,
            crl: 0x4444_4444,
            crh: 0x4444_4444,
            odr: 0,
            lckr: 0,
            locked: 0,
            lock_stage: 0,
            external: 0,
            driven: 0,
        }
    }

    pub fn reset(&mut self) {
        let index = self.index;
        *self = GpioPort::new(index);
    }

    /// Port index (0 = port A).
    pub fn index(&self) -> u8 {
        self.index
    }

    /// Configuration bits of one pin: `(cnf, mode)`.
    pub fn config(&self, pin: u8) -> (u8, u8) {
        let register = if pin < 8 { self.crl } else { self.crh };
        let shift = u32::from(pin % 8) * 4;
        (
            ((register >> (shift + 2)) & 0b11) as u8,
            ((register >> shift) & 0b11) as u8,
        )
    }

    /// True when the pin is configured as an output.
    pub fn is_output(&self, pin: u8) -> bool {
        self.config(pin).1 != 0
    }

    /// True when the configuration is locked through `LCKR`.
    pub fn is_locked(&self, pin: u8) -> bool {
        self.locked & (1 << pin) != 0
    }

    /// Level driven by the output latch, regardless of the pin direction.
    pub fn output_level(&self, pin: u8) -> bool {
        self.odr & (1 << pin) != 0
    }

    /// Level the input data register reports for `pin`.
    pub fn input_level(&self, pin: u8) -> bool {
        let (cnf, mode) = self.config(pin);
        if mode != 0 {
            // Output (push-pull, open-drain or alternate function).
            return self.output_level(pin);
        }
        if self.driven & (1 << pin) != 0 {
            // An external driver overrides the weak internal pull resistor.
            return self.external & (1 << pin) != 0;
        }
        match cnf {
            // Input with pull-up / pull-down: the latch selects the pull.
            0b10 => self.output_level(pin),
            // Floating or analog input without a driver reads low.
            _ => false,
        }
    }

    /// Drives the level seen by an input pin, as an external device would.
    pub fn set_external_level(&mut self, pin: u8, high: bool) {
        self.driven |= 1 << pin;
        if high {
            self.external |= 1 << pin;
        } else {
            self.external &= !(1 << pin);
        }
    }

    /// Releases a pin driven by [`GpioPort::set_external_level`].
    pub fn release_pin(&mut self, pin: u8) {
        self.driven &= !(1 << pin);
        self.external &= !(1 << pin);
    }

    /// Current value of `IDR`.
    pub fn idr(&self) -> u32 {
        let mut value = 0u32;
        for pin in 0..16u8 {
            if self.input_level(pin) {
                value |= 1 << pin;
            }
        }
        value
    }

    /// Current value of `ODR`.
    pub fn odr(&self) -> u32 {
        self.odr
    }

    fn update_odr(&mut self, cx: &Ctx, new_odr: u32) {
        let changed = self.odr ^ new_odr;
        self.odr = new_odr & 0xFFFF;
        for pin in 0..16u8 {
            if changed & (1 << pin) != 0 && self.is_output(pin) {
                cx.emit(Event::PinOutput {
                    port: self.index,
                    pin,
                    high: new_odr & (1 << pin) != 0,
                });
            }
        }
    }

    fn write_config(&mut self, _cx: &Ctx, offset: u32, width: AccessWidth, value: u32) {
        let (current, first_pin) = if offset < reg::CRH {
            (self.crl, 0u8)
        } else {
            (self.crh, 8u8)
        };
        let mut updated = merge(current, offset & 3, width, value);
        for pin in 0..8u8 {
            if self.is_locked(first_pin + pin) {
                let shift = u32::from(pin) * 4;
                let mask = 0xF << shift;
                updated = (updated & !mask) | (current & mask);
            }
        }
        if first_pin == 0 {
            self.crl = updated;
        } else {
            self.crh = updated;
        }
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::CRL => self.crl,
            reg::CRH => self.crh,
            reg::IDR => self.idr(),
            reg::ODR => self.odr,
            reg::BSRR => 0,
            reg::BRR => 0,
            reg::LCKR => self.lckr,
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
            reg::CRL | reg::CRH => self.write_config(cx, offset, width, value),
            reg::IDR => {}
            reg::ODR => {
                let new_odr = merge(self.odr, offset, width, value);
                self.update_odr(cx, new_odr);
            }
            reg::BSRR => {
                let set = value & 0xFFFF;
                let reset = (value >> 16) & 0xFFFF;
                let new_odr = (self.odr | set) & !reset;
                self.update_odr(cx, new_odr);
            }
            reg::BRR => {
                let new_odr = self.odr & !(value & 0xFFFF);
                self.update_odr(cx, new_odr);
            }
            reg::LCKR => {
                if value & LCKK != 0 {
                    self.lckr = (value & 0xFFFF) | LCKK;
                    self.lock_stage = 1;
                } else if self.lock_stage == 1 {
                    self.locked |= self.lckr & 0xFFFF;
                    self.lock_stage = 2;
                    self.lckr = value & 0xFFFF;
                } else {
                    self.lckr = value & 0xFFFF;
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

    pub fn advance(&mut self, _cx: &Ctx, _elapsed_ns: u64) {}
}

crate::impl_peripheral!(GpioPort, "GPIO");

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::default())
    }

    /// Builds a `CRL` value with `pin` configured as a 2 MHz push-pull output.
    fn crl_with_output(pin: u8) -> u32 {
        0x4444_4444 & !(0xF << (pin * 4)) | (0x2 << (pin * 4))
    }

    #[test]
    fn reset_configuration_is_floating_input() {
        let mut port = GpioPort::new(0);
        assert_eq!(port.config(0), (1, 0));
        assert_eq!(
            port.read_reg(&ctx(), reg::CRL, AccessWidth::Word).unwrap(),
            0x4444_4444
        );
    }

    #[test]
    fn push_pull_output_drives_idr() {
        let cx = ctx();
        let mut port = GpioPort::new(0);
        // PA5 push-pull output, 2 MHz.
        port.write_reg(&cx, reg::CRL, AccessWidth::Word, crl_with_output(5))
            .unwrap();
        port.write_reg(&cx, reg::BSRR, AccessWidth::Word, 1 << 5)
            .unwrap();
        assert!(port.is_output(5));
        assert!(port.input_level(5));
        assert_eq!(
            port.read_reg(&cx, reg::IDR, AccessWidth::Word).unwrap(),
            1 << 5
        );

        port.write_reg(&cx, reg::BSRR, AccessWidth::Word, 1 << (5 + 16))
            .unwrap();
        assert!(!port.input_level(5));
        assert_eq!(port.idr(), 0);
    }

    #[test]
    fn floating_input_follows_external_level() {
        let cx = ctx();
        let mut port = GpioPort::new(0);
        assert!(!port.input_level(0));
        port.set_external_level(0, true);
        assert!(port.input_level(0));
        assert_eq!(port.read_reg(&cx, reg::IDR, AccessWidth::Half).unwrap(), 1);
    }

    #[test]
    fn pull_up_input_reads_high_when_odr_bit_is_set() {
        let cx = ctx();
        let mut port = GpioPort::new(1);
        // PB1 input with pull-up/down: CNF = 10, MODE = 00 -> nibble 0b1000.
        port.write_reg(&cx, reg::CRL, AccessWidth::Word, 0x4444_4484)
            .unwrap();
        assert_eq!(port.config(1), (0b10, 0));
        assert!(!port.input_level(1), "ODR bit clear selects pull-down");
        port.write_reg(&cx, reg::ODR, AccessWidth::Word, 0b10)
            .unwrap();
        assert!(port.input_level(1), "ODR bit set selects pull-up");
    }

    #[test]
    fn output_changes_are_reported_as_events() {
        let cx = ctx();
        let mut port = GpioPort::new(0);
        port.write_reg(&cx, reg::CRL, AccessWidth::Word, crl_with_output(5))
            .unwrap();
        cx.take_events();
        port.write_reg(&cx, reg::BSRR, AccessWidth::Word, 1 << 5)
            .unwrap();
        assert_eq!(
            cx.take_events(),
            vec![Event::PinOutput {
                port: 0,
                pin: 5,
                high: true
            }]
        );
    }

    #[test]
    fn lock_sequence_freezes_configuration() {
        let cx = ctx();
        let mut port = GpioPort::new(0);
        port.write_reg(&cx, reg::LCKR, AccessWidth::Word, (1 << 5) | LCKK)
            .unwrap();
        port.write_reg(&cx, reg::LCKR, AccessWidth::Word, 1 << 5)
            .unwrap();
        assert!(port.is_locked(5));

        let before = port.config(5);
        port.write_reg(&cx, reg::CRL, AccessWidth::Word, crl_with_output(5))
            .unwrap();
        assert_eq!(port.config(5), before, "locked pin keeps its configuration");
        port.write_reg(&cx, reg::CRL, AccessWidth::Word, crl_with_output(0))
            .unwrap();
        assert_eq!(
            port.config(0),
            (0b00, 0b10),
            "unlocked pin follows the write"
        );
    }
}

impl GpioPort {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        let mut outputs = 0;
        let mut analog = 0;
        let mut alternate = 0;
        for pin in 0..16u8 {
            let (cnf, mode) = self.config(pin);
            if mode != 0 {
                outputs += 1;
                if cnf & 0b10 != 0 {
                    alternate += 1;
                }
            } else if cnf == 0b11 {
                analog += 1;
            }
        }
        let mut high = 0;
        for pin in 0..16u8 {
            if self.input_level(pin) {
                high += 1;
            }
        }
        vec![
            ("输出引脚", format!("{outputs}（其中 {alternate} 个复用）")),
            ("模拟输入", format!("{analog}")),
            ("高电平引脚", format!("{high}")),
            ("ODR", crate::status::hex(self.odr)),
            ("IDR", crate::status::hex(self.idr())),
            ("CRL", crate::status::hex(self.crl)),
            ("CRH", crate::status::hex(self.crh)),
        ]
    }
}