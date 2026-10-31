//! MAX7219 serial 8x8 LED matrix controller.
//!
//! The part receives 16 bit words through a three wire SPI port (`DIN`, `CLK`,
//! `LOAD/CS`): the first byte is the register address, the second the value.
//! Data is latched on the rising edge of `LOAD`. Several parts can be chained;
//! the first word shifted in ends up in the last chip of the chain.
//!
//! The model consumes the events the SoC produces, exactly as a wiring harness
//! would:
//!
//! * [`Event::SpiTransfer`] for the port the device sits on (one event per
//!   shifted byte),
//! * [`Event::PinOutput`] for the `LOAD` line, which delimits transactions.
//!
//! `DOUT` is modelled too: it echoes the word received 16 bits earlier, which
//! is what the `MAX7219_EchoTest()` diagnostic of the sample firmware reads
//! back through `MISO`.

use gsim_core::Event;

/// Width and height of one chip's matrix.
pub const MATRIX_SIZE: usize = 8;

/// Register addresses of the controller.
pub mod register {
    pub const NOOP: u8 = 0x00;
    pub const DIGIT0: u8 = 0x01;
    pub const DIGIT_LAST: u8 = 0x08;
    pub const DECODE_MODE: u8 = 0x09;
    pub const INTENSITY: u8 = 0x0A;
    pub const SCAN_LIMIT: u8 = 0x0B;
    pub const SHUTDOWN: u8 = 0x0C;
    pub const DISPLAY_TEST: u8 = 0x0F;
}

/// One MAX7219 die of the chain.
#[derive(Clone, Debug)]
pub struct ChipState {
    registers: [u8; 16],
}

impl Default for ChipState {
    fn default() -> Self {
        ChipState { registers: [0; 16] }
    }
}

impl ChipState {
    pub fn register(&self, address: u8) -> u8 {
        self.registers[usize::from(address & 0x0F)]
    }

    /// Rows enabled by `SCAN_LIMIT` (register value + 1, clipped to eight).
    pub fn scanned_rows(&self) -> usize {
        (usize::from(self.register(register::SCAN_LIMIT) & 0x07) + 1).min(MATRIX_SIZE)
    }

    /// Brightness set through `INTENSITY` (0..=15).
    pub fn intensity(&self) -> u8 {
        self.register(register::INTENSITY) & 0x0F
    }

    /// `false` while the shutdown register holds zero.
    pub fn enabled(&self) -> bool {
        self.register(register::SHUTDOWN) & 0x01 != 0
    }

    pub fn display_test(&self) -> bool {
        self.register(register::DISPLAY_TEST) & 0x01 != 0
    }

    pub fn decode_mode(&self) -> u8 {
        self.register(register::DECODE_MODE)
    }

    /// Row bitmap of digit `row` (0 = `DIGIT0`), bit 7 is the leftmost column.
    pub fn row(&self, row: usize) -> u8 {
        self.register(register::DIGIT0 + row as u8)
    }

    /// Visible pixels of this chip as `rows[row][column]`.
    pub fn pixels(&self) -> [[bool; MATRIX_SIZE]; MATRIX_SIZE] {
        let mut pixels = [[false; MATRIX_SIZE]; MATRIX_SIZE];
        let test = self.display_test();
        let enabled = self.enabled();
        let rows = self.scanned_rows();
        for (row, line) in pixels.iter_mut().enumerate() {
            if !test && (!enabled || row >= rows) {
                continue;
            }
            let bits = if test { 0xFF } else { self.row(row) };
            for (column, pixel) in line.iter_mut().enumerate() {
                *pixel = bits & (1 << (7 - column)) != 0;
            }
        }
        pixels
    }
}

/// A MAX7219 chain attached to one SPI port.
#[derive(Clone, Debug)]
pub struct Max7219 {
    unit: u8,
    cs_port: u8,
    cs_pin: u8,
    chips: Vec<ChipState>,
    /// `true` while `LOAD` is high, which is the idle state between words.
    load_high: bool,
    shift: u64,
    bits_in_shift: u32,
    /// Bytes the controller received since power-on (capped at two).
    last_bytes: [u8; 2],
    pending_miso: u16,
    frames: u64,
    words: u64,
}

impl Max7219 {
    /// Creates a chain of `chips` dies on SPI `unit`, latched by `cs_port`/`cs_pin`.
    pub fn new(unit: u8, cs_port: u8, cs_pin: u8, chips: usize) -> Self {
        Max7219 {
            unit,
            cs_port,
            cs_pin,
            chips: vec![ChipState::default(); chips.max(1)],
            load_high: true,
            shift: 0,
            bits_in_shift: 0,
            last_bytes: [0; 2],
            pending_miso: 0,
            frames: 0,
            words: 0,
        }
    }

    /// The chain the CubeMX sample firmware uses: SPI1, `PA4` as LOAD, one die.
    pub fn firmware_default() -> Self {
        Max7219::new(0, 0, 4, 1)
    }

    pub fn unit(&self) -> u8 {
        self.unit
    }

    pub fn cs_pin(&self) -> (u8, u8) {
        (self.cs_port, self.cs_pin)
    }

    pub fn chips(&self) -> &[ChipState] {
        &self.chips
    }

    /// Number of SPI bytes received so far.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Number of 16 bit words latched by a `LOAD` rising edge.
    pub fn words(&self) -> u64 {
        self.words
    }

    /// Value the part shifts out on `DOUT` during the next SPI frame.
    pub fn miso(&self) -> u16 {
        self.pending_miso
    }

    /// Consumes one peripheral event.
    pub fn on_event(&mut self, event: &Event) {
        match event {
            Event::SpiTransfer { unit, tx, .. } if *unit == self.unit => self.shift_frame(*tx),
            Event::PinOutput { port, pin, high }
                if *port == self.cs_port && *pin == self.cs_pin =>
            {
                self.set_load(*high)
            }
            _ => {}
        }
    }

    /// Drives the `LOAD` line directly.
    pub fn set_load(&mut self, high: bool) {
        if high == self.load_high {
            return;
        }
        self.load_high = high;
        if high {
            self.latch();
        } else {
            self.shift = 0;
            self.bits_in_shift = 0;
        }
    }

    fn shift_frame(&mut self, value: u16) {
        self.frames += 1;
        let byte = (value & 0xFF) as u8;
        // DOUT repeats the bits received 16 bits ago, i.e. the byte before the
        // one that just arrived.
        self.pending_miso = u16::from(self.last_bytes[1]);
        self.last_bytes = [self.last_bytes[1], byte];

        if self.load_high {
            // Frames shifted while LOAD is high are ignored by the part.
            return;
        }
        let bits = if value > 0xFF { 16u32 } else { 8 };
        let mask = (1u64 << bits) - 1;
        self.shift = (self.shift << bits) | (u64::from(value) & mask);
        self.bits_in_shift = (self.bits_in_shift + bits).min(64);
    }

    fn latch(&mut self) {
        let words = (self.bits_in_shift / 16) as usize;
        for word_index in 0..words {
            let shift = (words - 1 - word_index) * 16;
            let word = ((self.shift >> shift) & 0xFFFF) as u16;
            // The first word shifted in belongs to the last die of the chain.
            let chip = self.chips.len() - 1 - word_index.min(self.chips.len() - 1);
            self.write_word(chip, word);
        }
        self.shift = 0;
        self.bits_in_shift = 0;
    }

    fn write_word(&mut self, chip: usize, word: u16) {
        let address = (word >> 8) as u8 & 0x0F;
        let data = (word & 0xFF) as u8;
        self.words += 1;
        if address == register::NOOP {
            return;
        }
        if let Some(die) = self.chips.get_mut(chip) {
            die.registers[usize::from(address)] = data;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn transfer(device: &mut Max7219, bytes: &[u8]) {
        device.set_load(false);
        for byte in bytes {
            let frame = Event::SpiTransfer {
                unit: 0,
                tx: u16::from(*byte),
                rx: 0,
            };
            device.on_event(&frame);
        }
        device.set_load(true);
    }

    #[test]
    fn a_word_written_between_load_edges_configures_the_chip() {
        let mut device = Max7219::new(0, 0, 4, 1);
        transfer(&mut device, &[register::SHUTDOWN, 0x01]);
        transfer(&mut device, &[register::SCAN_LIMIT, 0x07]);
        transfer(&mut device, &[register::INTENSITY, 0x07]);

        let chip = &device.chips()[0];
        assert!(chip.enabled());
        assert_eq!(chip.scanned_rows(), 8);
        assert_eq!(chip.intensity(), 7);
        assert_eq!(device.words(), 3);
    }

    #[test]
    fn frames_shifted_while_load_is_high_are_discarded() {
        let mut device = Max7219::new(0, 0, 4, 1);
        for byte in [register::SHUTDOWN, 0x01] {
            device.on_event(&Event::SpiTransfer {
                unit: 0,
                tx: u16::from(byte),
                rx: 0,
            });
        }
        assert_eq!(device.words(), 0);
        assert!(!device.chips()[0].enabled());
    }

    #[test]
    fn digit_registers_are_rendered_into_the_matrix() {
        let mut device = Max7219::new(0, 0, 4, 1);
        transfer(&mut device, &[register::SHUTDOWN, 0x01]);
        transfer(&mut device, &[register::SCAN_LIMIT, 0x07]);
        transfer(&mut device, &[register::DISPLAY_TEST, 0x00]);
        transfer(&mut device, &[register::DIGIT0, 0x81]);
        transfer(&mut device, &[register::DIGIT0 + 7, 0x18]);

        let pixels = device.chips()[0].pixels();
        assert!(pixels[0][0], "bit 7 of row 0 is the top left pixel");
        assert!(pixels[0][7]);
        assert!(!pixels[0][3]);
        assert!(pixels[7][3]);
        assert!(pixels[7][4]);
    }

    #[test]
    fn shutdown_blanks_the_matrix_and_display_test_fills_it() {
        let mut device = Max7219::new(0, 0, 4, 1);
        transfer(&mut device, &[register::DIGIT0, 0xFF]);
        assert!(device.chips()[0].pixels()[0].iter().all(|p| !p));

        transfer(&mut device, &[register::SHUTDOWN, 0x01]);
        assert!(device.chips()[0].pixels()[0].iter().all(|p| *p));

        transfer(&mut device, &[register::DISPLAY_TEST, 0x01]);
        assert!(device.chips()[0].pixels()[3].iter().all(|p| *p));
    }

    #[test]
    fn scan_limit_blanks_the_rows_above_it() {
        let mut device = Max7219::new(0, 0, 4, 1);
        transfer(&mut device, &[register::SHUTDOWN, 0x01]);
        transfer(&mut device, &[register::SCAN_LIMIT, 0x02]);
        for row in 0..8u8 {
            transfer(&mut device, &[register::DIGIT0 + row, 0xFF]);
        }
        let pixels = device.chips()[0].pixels();
        assert!(pixels[2].iter().all(|pixel| *pixel));
        assert!(pixels[3].iter().all(|pixel| !*pixel));
    }

    #[test]
    fn dout_echoes_the_word_received_sixteen_bits_earlier() {
        let mut device = Max7219::new(0, 0, 4, 1);
        device.set_load(false);
        let mut echoed = Vec::new();
        for byte in [0x0Au8, 0x07, 0x00, 0x00] {
            echoed.push(device.miso());
            device.on_event(&Event::SpiTransfer {
                unit: 0,
                tx: u16::from(byte),
                rx: 0,
            });
        }
        device.set_load(true);
        // Two frames of delay: the NOOP transaction reads back 0x0A 0x07.
        let echoes: Vec<u16> = echoed.iter().map(|byte| *byte).collect();
        assert_eq!(echoes[1..], [0x00, 0x0A, 0x07]);
    }

    #[test]
    fn chained_chips_take_the_first_word() {
        let mut device = Max7219::new(0, 0, 4, 2);
        transfer(
            &mut device,
            &[register::DIGIT0, 0x11, register::DIGIT0, 0x22],
        );
        assert_eq!(device.chips()[0].row(0), 0x22);
        assert_eq!(device.chips()[1].row(0), 0x11);
    }
}
