//! Virtual UART peer: a serial terminal attached to one USART of the MCU.
//!
//! The part is a pure data endpoint, exactly like a USB-serial adapter sitting
//! on the board:
//!
//! * every byte the controller shifts out on its `TX` line arrives here through
//!   [`Event::UartTx`] and is appended to a printable log,
//! * bytes typed into the terminal are queued and handed back to the front end
//!   with [`Uart::next_outgoing`], which injects them into the controller's
//!   receiver.
//!
//! Which controller pin is actually wired to the terminal is a property of the
//! board, not of the part: the front end decides whether an event reaches this
//! device, and whether queued bytes may be injected back.

use std::collections::VecDeque;

use gsim_core::Event;

/// How many received bytes the log keeps before it is trimmed.
pub const RX_LOG_LIMIT: usize = 8_192;
/// How many received bytes are kept raw for hexadecimal inspection.
pub const RAW_LIMIT: usize = 4_096;

/// A serial terminal attached to one USART.
#[derive(Clone, Debug)]
pub struct Uart {
    unit: u8,
    rx_text: String,
    raw: VecDeque<u8>,
    outgoing: VecDeque<u8>,
    rx_bytes: u64,
    tx_bytes: u64,
    dropped: u64,
}

impl Default for Uart {
    fn default() -> Self {
        Uart::new(0)
    }
}

impl Uart {
    /// Creates a terminal on USART `unit` (0 = USART1).
    pub fn new(unit: u8) -> Self {
        Uart {
            unit,
            rx_text: String::new(),
            raw: VecDeque::new(),
            outgoing: VecDeque::new(),
            rx_bytes: 0,
            tx_bytes: 0,
            dropped: 0,
        }
    }

    /// The terminal the CubeMX firmware talks to: USART1 at 115200 8N1.
    pub fn firmware_default() -> Self {
        Uart::new(0)
    }

    pub fn unit(&self) -> u8 {
        self.unit
    }

    /// Moves the terminal to another USART; the log is kept.
    pub fn set_unit(&mut self, unit: u8) {
        self.unit = unit;
    }

    /// Bytes received from the controller so far.
    pub fn rx_bytes(&self) -> u64 {
        self.rx_bytes
    }

    /// Bytes sent towards the controller so far.
    pub fn tx_bytes(&self) -> u64 {
        self.tx_bytes
    }

    /// Bytes queued but not yet injected into the controller.
    pub fn pending_tx(&self) -> usize {
        self.outgoing.len()
    }

    /// Printable view of what the controller transmitted.
    pub fn rx_text(&self) -> &str {
        &self.rx_text
    }

    /// Last received bytes in arrival order (capped at [`RAW_LIMIT`]).
    pub fn raw(&self) -> impl Iterator<Item = u8> + '_ {
        self.raw.iter().copied()
    }

    /// Consumes one peripheral event; other units are ignored.
    pub fn on_event(&mut self, event: &Event) {
        if let Event::UartTx { unit, byte } = event {
            if *unit == self.unit {
                self.receive(*byte);
            }
        }
    }

    /// Appends one byte received from the controller.
    pub fn receive(&mut self, byte: u8) {
        self.rx_bytes += 1;
        if self.raw.len() == RAW_LIMIT {
            self.raw.pop_front();
        }
        self.raw.push_back(byte);
        self.push_text(byte);
    }

    fn push_text(&mut self, byte: u8) {
        match byte {
            b'\r' => {}
            b'\n' => self.rx_text.push('\n'),
            b'\t' => self.rx_text.push(' '),
            0x20..=0x7E => self.rx_text.push(char::from(byte)),
            _ => self.rx_text.push('·'),
        }
        if self.rx_text.len() > RX_LOG_LIMIT {
            let cut = self.rx_text.len() - RX_LOG_LIMIT / 2;
            // Keep the tail, on a character boundary.
            let mut start = cut;
            while start < self.rx_text.len() && !self.rx_text.is_char_boundary(start) {
                start += 1;
            }
            self.rx_text.replace_range(0..start, "");
        }
    }

    /// Queues text typed into the terminal. Returns the number of bytes queued.
    pub fn send(&mut self, text: &str) -> usize {
        self.send_bytes(text.as_bytes())
    }

    /// Queues raw bytes towards the controller.
    pub fn send_bytes(&mut self, bytes: &[u8]) -> usize {
        let mut queued = 0;
        for byte in bytes {
            if self.outgoing.len() >= 4_096 {
                self.dropped += 1;
                continue;
            }
            self.outgoing.push_back(*byte);
            queued += 1;
        }
        queued
    }

    /// Pops the next byte the controller should receive.
    pub fn next_outgoing(&mut self) -> Option<u8> {
        let byte = self.outgoing.pop_front()?;
        self.tx_bytes += 1;
        Some(byte)
    }

    /// Puts a byte back in front of the queue, for when the receiver was busy.
    pub fn requeue(&mut self, byte: u8) {
        self.outgoing.push_front(byte);
    }

    /// Bytes that were thrown away because the queue was full.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Clears the log and the transmit queue.
    pub fn clear(&mut self) {
        self.rx_text.clear();
        self.raw.clear();
        self.outgoing.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn byte(unit: u8, byte: u8) -> Event {
        Event::UartTx { unit, byte }
    }

    #[test]
    fn bytes_from_the_configured_unit_become_readable_text() {
        let mut uart = Uart::new(0);
        for value in b"AT+OK\r\n" {
            uart.on_event(&byte(0, *value));
        }
        assert_eq!(uart.rx_text(), "AT+OK\n");
        assert_eq!(uart.rx_bytes(), 7);
        assert_eq!(uart.raw().collect::<Vec<_>>(), b"AT+OK\r\n");
    }

    #[test]
    fn traffic_of_other_units_is_ignored() {
        let mut uart = Uart::new(1);
        uart.on_event(&byte(0, b'x'));
        assert_eq!(uart.rx_bytes(), 0);
        uart.on_event(&byte(1, b'y'));
        assert_eq!(uart.rx_text(), "y");
    }

    #[test]
    fn control_bytes_are_visualised_without_breaking_the_log() {
        let mut uart = Uart::new(0);
        for value in [0x01u8, b'\t', 0x80] {
            uart.receive(value);
        }
        assert_eq!(uart.rx_text(), "· ·");
    }

    #[test]
    fn typed_text_is_queued_and_drained_in_order() {
        let mut uart = Uart::new(0);
        assert_eq!(uart.send("hi\r\n"), 4);
        assert_eq!(uart.pending_tx(), 4);
        let mut drained = Vec::new();
        while let Some(value) = uart.next_outgoing() {
            drained.push(value);
        }
        assert_eq!(drained, b"hi\r\n");
        assert_eq!(uart.tx_bytes(), 4);
        assert_eq!(uart.pending_tx(), 0);
        assert!(uart.next_outgoing().is_none());
    }

    #[test]
    fn the_queue_is_bounded_instead_of_growing_forever() {
        let mut uart = Uart::new(0);
        uart.send_bytes(&vec![b'a'; 5_000]);
        assert_eq!(uart.pending_tx(), 4_096);
        assert_eq!(uart.dropped(), 5_000 - 4_096);
    }

    #[test]
    fn moving_the_terminal_keeps_the_log_but_changes_the_source() {
        let mut uart = Uart::new(0);
        uart.on_event(&byte(0, b'A'));
        uart.set_unit(2);
        uart.on_event(&byte(0, b'B'));
        uart.on_event(&byte(2, b'C'));
        assert_eq!(uart.rx_text(), "AC");
    }

    #[test]
    fn clear_resets_the_log_and_the_queue() {
        let mut uart = Uart::new(0);
        uart.receive(b'x');
        uart.send("pending");
        uart.clear();
        assert_eq!(uart.rx_text(), "");
        assert_eq!(uart.pending_tx(), 0);
        assert_eq!(uart.raw().count(), 0);
    }
}
