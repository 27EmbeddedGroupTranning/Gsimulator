//! Access context passed to every peripheral operation.
//!
//! A peripheral never touches the NVIC, the tracer or another peripheral
//! directly. It reports side effects as [`Event`]s into the [`Ctx`] it was
//! given; the SoC drains them once the current operation or simulation step is
//! complete. That keeps peripheral code free of global state and makes the
//! whole model deterministic and easy to test.

use std::cell::RefCell;

use crate::can::{CanDirection, CanFrame};
use crate::irq::Irq;

/// Clock tree snapshot handed to peripherals.
///
/// The SoC refreshes it from the RCC model before every step, so peripheral
/// code never has to reason about the RCC itself. A clock of `0` Hz means the
/// peripheral is not clocked and must not advance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clocks {
    pub sysclk_hz: u32,
    pub hclk_hz: u32,
    pub pclk1_hz: u32,
    pub pclk2_hz: u32,
    pub tim1_hz: u32,
    pub tim2_hz: u32,
    pub adc_hz: u32,
    pub rtc_hz: u32,
}

impl Clocks {
    /// Power-on clock tree: HSI 8 MHz feeding the CPU directly.
    pub const HSI_8MHZ: Clocks = Clocks {
        sysclk_hz: 8_000_000,
        hclk_hz: 8_000_000,
        pclk1_hz: 8_000_000,
        pclk2_hz: 8_000_000,
        tim1_hz: 8_000_000,
        tim2_hz: 8_000_000,
        adc_hz: 4_000_000,
        rtc_hz: 0,
    };
}

impl Default for Clocks {
    fn default() -> Self {
        Clocks::HSI_8MHZ
    }
}

/// Side effect reported by a peripheral.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    /// An external interrupt line changed level.
    Irq { irq: Irq, high: bool },
    /// A Cortex-M system exception became pending (SysTick, PendSV, ...).
    Exception { number: u16 },
    /// A USART shifted a byte out.
    UartTx { unit: u8, byte: u8 },
    /// A SPI completed one frame.
    SpiTransfer { unit: u8, tx: u16, rx: u16 },
    /// A GPIO pin configured as output changed level.
    PinOutput { port: u8, pin: u8, high: bool },
    /// A CAN controller moved one frame across its pin interface or looped a
    /// frame back internally.
    CanFrame {
        unit: u8,
        direction: CanDirection,
        frame: CanFrame,
    },
    /// Free-form note for tracing (watchdog expiry, clock switch, ...).
    Note {
        source: &'static str,
        message: &'static str,
        detail: u32,
    },
}

/// Consumer of [`Event`]s, registered on the SoC.
pub trait EventSink {
    fn on_event(&mut self, event: &Event);
}

impl<F> EventSink for F
where
    F: FnMut(&Event),
{
    fn on_event(&mut self, event: &Event) {
        self(event)
    }
}

/// Per-operation context: current time, clock snapshot and event collector.
#[derive(Debug)]
pub struct Ctx {
    /// Simulation time in nanoseconds since power-on.
    now_ns: u64,
    clocks: Clocks,
    events: RefCell<Vec<Event>>,
}

impl Ctx {
    pub fn new(now_ns: u64, clocks: Clocks) -> Self {
        Ctx {
            now_ns,
            clocks,
            events: RefCell::new(Vec::new()),
        }
    }

    pub fn now_ns(&self) -> u64 {
        self.now_ns
    }

    pub fn clocks(&self) -> &Clocks {
        &self.clocks
    }

    pub fn irq(&self, irq: Irq, high: bool) {
        self.emit(Event::Irq { irq, high });
    }

    pub fn exception(&self, number: u16) {
        self.emit(Event::Exception { number });
    }

    pub fn emit(&self, event: Event) {
        self.events.borrow_mut().push(event);
    }

    pub fn note(&self, source: &'static str, message: &'static str, detail: u32) {
        self.emit(Event::Note {
            source,
            message,
            detail,
        });
    }

    /// Removes and returns all events collected so far.
    pub fn take_events(&self) -> Vec<Event> {
        std::mem::take(&mut *self.events.borrow_mut())
    }

    /// True when nothing was reported since the last drain.
    pub fn is_idle(&self) -> bool {
        self.events.borrow().is_empty()
    }
}

impl Default for Ctx {
    fn default() -> Self {
        Ctx::new(0, Clocks::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::irq::lines;

    #[test]
    fn events_are_collected_until_drained() {
        let cx = Ctx::default();
        assert!(cx.is_idle());
        cx.irq(lines::USART1, true);
        cx.note("test", "hello", 7);
        assert!(!cx.is_idle());
        let events = cx.take_events();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0],
            Event::Irq {
                irq: lines::USART1,
                high: true
            }
        );
        assert!(cx.is_idle());
    }

    #[test]
    fn clock_snapshot_defaults_to_hsi() {
        let cx = Ctx::default();
        assert_eq!(cx.clocks().hclk_hz, 8_000_000);
    }
}
