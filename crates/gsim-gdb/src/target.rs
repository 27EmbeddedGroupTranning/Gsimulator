//! The simulated STM32F103 as a debugger sees it.
//!
//! Everything a probe would do to the real chip lives here: halt and resume the
//! core, read and write registers, peek and poke memory, place breakpoints and
//! watchpoints, and program the flash through the on-chip controller the same
//! way an ST-Link does (`KEYR` unlock, `CR.PG`/`CR.PER`, `AR`, `CR.STRT`).

use std::cell::RefCell;
use std::io::Write;
use std::path::Path;
use std::rc::Rc;

use gsim_core::{AccessWidth, Event};
use gsim_cpu::{Cpu, ElfImage, StopReason};
use gsim_stm32f1::map::{self, flash as flash_reg};
use gsim_stm32f1::memory::{FLASH_KEY1, FLASH_KEY2};
use gsim_stm32f1::{Memory, Soc};

use crate::description;

/// `CR.PG`: half-word programming enabled.
const FLASH_CR_PG: u32 = 1 << 0;
/// `CR.PER`: page erase enabled.
const FLASH_CR_PER: u32 = 1 << 1;
/// `CR.STRT`: start the latched erase.
const FLASH_CR_STRT: u32 = 1 << 6;
/// `CR.LOCK`: the controller ignores programming requests.
const FLASH_CR_LOCK: u32 = 1 << 7;
/// `SR.EOP`: the last operation finished.
const FLASH_SR_EOP: u32 = 1 << 5;
/// `SR.WRPRTERR`: a write was attempted on a locked or protected page.
const FLASH_SR_WRPRTERR: u32 = 1 << 4;

/// Signals the server reports to GDB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Signal {
    Interrupt,
    Trap,
    Segv,
}

impl Signal {
    /// Signal number understood by the remote protocol.
    pub fn number(self) -> u8 {
        match self {
            Signal::Interrupt => 2,
            Signal::Trap => 5,
            Signal::Segv => 11,
        }
    }

    /// Name used in the server's diagnostics.
    pub fn name(self) -> &'static str {
        match self {
            Signal::Interrupt => "SIGINT",
            Signal::Trap => "SIGTRAP",
            Signal::Segv => "SIGSEGV",
        }
    }
}

/// Why the core stopped, in the terms GDB speaks.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Stop {
    /// A breakpoint was reached.
    Breakpoint { address: u32, hardware: bool },
    /// The firmware executed `BKPT`.
    Break,
    /// The core reached `WFI` with no wake-up source left.
    Halted,
    /// The device reset itself while running.
    Reset,
    /// The debugger asked for a halt (control-C).
    Halt,
    /// A watched memory location changed.
    Watchpoint { address: u32 },
}

impl Stop {
    /// Signal the remote protocol reports for this stop.
    pub fn signal(&self) -> Signal {
        match self {
            Stop::Halt => Signal::Interrupt,
            _ => Signal::Trap,
        }
    }
}

/// Kind of breakpoint or watchpoint a `Z`/`z` packet refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointKind {
    /// A breakpoint GDB would implement by patching the instruction stream.
    Software,
    /// A breakpoint a probe implements with a hardware comparator.
    Hardware,
    /// Watchpoint that fires on writes.
    Write,
    /// Watchpoint that fires on reads.
    Read,
    /// Watchpoint that fires on any access.
    Access,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Breakpoint {
    address: u32,
    hardware: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Watchpoint {
    address: u32,
    length: u32,
}

/// The simulated chip plus the debug state attached to it.
pub struct Target {
    cpu: Cpu,
    image: Option<ElfImage>,
    breakpoints: Vec<Breakpoint>,
    watchpoints: Vec<Watchpoint>,
    watch_values: Vec<Vec<u8>>,
}

impl Target {
    /// Builds the chip model and wires the firmware's UART to stdout.
    pub fn new(hse_hz: u32, echo_uart: bool) -> Result<Self, String> {
        let soc = Rc::new(RefCell::new(Soc::with_hse(hse_hz)));
        if echo_uart {
            soc.borrow_mut().add_event_sink(Box::new(|event: &Event| {
                if let Event::UartTx { unit: 0, byte } = event {
                    let mut stdout = std::io::stdout();
                    let _ = stdout.write_all(&[*byte]);
                    let _ = stdout.flush();
                }
            }));
        }
        let cpu = Cpu::new(Rc::clone(&soc)).map_err(|error| error.to_string())?;
        Ok(Target {
            cpu,
            image: None,
            breakpoints: Vec::new(),
            watchpoints: Vec::new(),
            watch_values: Vec::new(),
        })
    }

    /// Shared handle to the device model.
    pub fn soc(&self) -> Rc<RefCell<Soc>> {
        self.cpu.soc()
    }

    /// Loads an ELF image into flash and resets the core from its vectors.
    pub fn load_elf(&mut self, path: &Path) -> Result<(), String> {
        let image = ElfImage::open(path).map_err(|error| error.to_string())?;
        self.cpu.load(&image).map_err(|error| error.to_string())?;
        self.image = Some(image);
        Ok(())
    }

    /// The image started with `--elf`, when there is one.
    pub fn image(&self) -> Option<&ElfImage> {
        self.image.as_ref()
    }

    /// Stack pointer and reset handler taken from the vector table.
    ///
    /// `None` while flash does not hold a firmware image.
    pub fn reset_vector(&self) -> Option<(u32, u32)> {
        let soc = self.cpu.soc();
        let soc = soc.borrow();
        let sp = soc.memory.read(map::FLASH_BASE, AccessWidth::Word).ok()?;
        let reset = soc
            .memory
            .read(map::FLASH_BASE + 4, AccessWidth::Word)
            .ok()?;
        let usable = sp != 0 && sp != u32::MAX && reset != 0 && reset != u32::MAX;
        usable.then_some((sp, reset & !1))
    }

    /// Attaches the way a probe does: the core stops in a defined state.
    ///
    /// A chip that has not executed anything yet starts from the vector table
    /// in flash; a later reconnect keeps the core where the session left it.
    pub fn attach(&mut self) {
        if self.cpu.instructions() == 0 && self.reset_vector().is_some() {
            let _ = self.cpu.reset();
        }
        self.cpu.halt();
    }

    /// Restarts the core from the vector table in flash.
    pub fn reset(&mut self) -> Result<(), String> {
        self.cpu.reset().map_err(|error| error.to_string())?;
        self.cpu.halt();
        Ok(())
    }

    /// Resets the core and every peripheral, like pulling the reset pin.
    pub fn hard_reset(&mut self) -> Result<(), String> {
        self.cpu.soc().borrow_mut().system_reset(0);
        self.reset()?;
        // The device reset was requested here, so do not report it again as a
        // stop reason when the core resumes.
        self.cpu.adopt_device_reset();
        Ok(())
    }

    /// Stops the core before its next instruction.
    pub fn halt(&mut self) {
        self.cpu.halt();
    }

    /// True while the core is not executing instructions.
    pub fn is_halted(&self) -> bool {
        self.cpu.halted()
    }

    /// Current program counter, without the Thumb bit.
    pub fn pc(&self) -> u32 {
        self.cpu.pc()
    }

    /// Reads the whole register file in `g` packet order.
    pub fn read_registers(&self) -> Vec<u32> {
        (0..description::REGISTER_COUNT)
            .map(|index| self.read_register(index))
            .collect()
    }

    /// Reads one register; indices outside the file read as zero.
    pub fn read_register(&self, index: usize) -> u32 {
        if index == description::XPSR_REGISTER {
            return self.cpu.xpsr();
        }
        self.cpu.register(index as u16).unwrap_or(0)
    }

    /// Writes the whole register file in `g` packet order.
    pub fn write_registers(&mut self, values: &[u32]) {
        for (index, value) in values.iter().enumerate() {
            self.write_register(index, *value);
        }
    }

    /// Writes one register; returns `false` for an unknown index.
    pub fn write_register(&mut self, index: usize, value: u32) -> bool {
        if index == description::XPSR_REGISTER {
            self.cpu.set_xpsr(value);
            return true;
        }
        self.cpu.set_register(index as u16, value)
    }

    /// Reads `length` bytes of the device address space.
    pub fn read_memory(&self, address: u32, length: u32) -> Result<Vec<u8>, String> {
        let Some(end) = address.checked_add(length) else {
            return Err(format!(
                "the read of 0x{address:08X} wraps the address space"
            ));
        };
        let mut data = Vec::with_capacity(length as usize);
        let mut address = address;
        while address < end {
            let left = end - address;
            let width = if address % 4 == 0 && left >= 4 {
                AccessWidth::Word
            } else if address % 2 == 0 && left >= 2 {
                AccessWidth::Half
            } else {
                AccessWidth::Byte
            };
            let value = self
                .cpu
                .read_memory(address, width)
                .map_err(|error| error.to_string())?;
            for index in 0..width.bytes() {
                data.push(((value >> (index * 8)) & 0xFF) as u8);
            }
            address += width.bytes();
        }
        Ok(data)
    }

    /// Writes bytes: flash is programmed through the controller, everything
    /// else goes straight through the bus.
    pub fn write_memory(&mut self, address: u32, data: &[u8]) -> Result<(), String> {
        let mut address = address;
        let mut offset = 0;
        while offset < data.len() {
            let room = self.room(address);
            let length = (data.len() - offset).min(room);
            let chunk = &data[offset..offset + length];
            if Memory::is_flash(address) {
                self.flash_write(address, chunk)?;
            } else {
                self.write_words(address, chunk)?;
            }
            offset += length;
            address += length as u32;
        }
        Ok(())
    }

    /// Bytes that can be touched before the access would cross a region border.
    fn room(&self, address: u32) -> usize {
        if Memory::is_flash(address) {
            (map::FLASH_BASE + map::FLASH_SIZE - address) as usize
        } else if Memory::is_sram(address) {
            (map::SRAM_BASE + map::SRAM_SIZE - address) as usize
        } else {
            // Peripherals, the core's system space and the bit-band aliases are
            // register windows: one access at a time. Addresses that are not
            // mapped at all are reported by the bus, not guessed at here.
            4
        }
    }

    fn write_words(&mut self, mut address: u32, data: &[u8]) -> Result<(), String> {
        let mut offset = 0;
        while offset < data.len() {
            let left = data.len() - offset;
            let width = if address % 4 == 0 && left >= 4 {
                AccessWidth::Word
            } else if address % 2 == 0 && left >= 2 {
                AccessWidth::Half
            } else {
                AccessWidth::Byte
            };
            let bytes = width.bytes() as usize;
            let mut value = 0u32;
            for index in 0..bytes {
                value |= u32::from(data[offset + index]) << (index * 8);
            }
            self.cpu
                .write_memory(address, width, value)
                .map_err(|error| error.to_string())?;
            offset += bytes;
            address += bytes as u32;
        }
        Ok(())
    }

    /// Erases every flash page that overlaps `address..address + length`.
    pub fn flash_erase(&mut self, address: u32, length: u32) -> Result<(), String> {
        if !Memory::is_flash(address) {
            return Err(format!("0x{address:08X} is not in the flash bank"));
        }
        let page_size = description::FLASH_PAGE_SIZE;
        let end = address.saturating_add(length.max(1));
        let mut page = address & !(page_size - 1);

        self.flash_unlock()?;
        while page < end {
            self.write_flash_register(flash_reg::CR, FLASH_CR_PER)?;
            self.write_flash_register(flash_reg::AR, page)?;
            self.write_flash_register(flash_reg::CR, FLASH_CR_PER | FLASH_CR_STRT)?;
            page += page_size;
        }
        self.write_flash_register(flash_reg::CR, 0)?;
        self.flash_lock();
        Ok(())
    }

    /// Programs `data` into erased flash and verifies the result.
    pub fn flash_write(&mut self, address: u32, data: &[u8]) -> Result<(), String> {
        if address % 2 != 0 {
            return Err(format!(
                "flash programs half words, so a write cannot start at 0x{address:08X}"
            ));
        }
        if data.is_empty() {
            return Ok(());
        }
        self.flash_unlock()?;
        self.write_flash_register(flash_reg::CR, FLASH_CR_PG)?;
        let mut cursor = address;
        for pair in data.chunks(2) {
            let halfword = u16::from_le_bytes([pair[0], pair.get(1).copied().unwrap_or(0xFF)]);
            self.cpu
                .write_memory(cursor, AccessWidth::Half, u32::from(halfword))
                .map_err(|error| error.to_string())?;
            cursor += 2;
        }
        self.write_flash_register(flash_reg::CR, 0)?;
        let status = self.read_flash_register(flash_reg::SR)?;
        self.flash_lock();
        if status & FLASH_SR_WRPRTERR != 0 {
            return Err(format!(
                "the flash controller rejected the write at 0x{address:08X}"
            ));
        }

        let verify = self.read_memory(address, data.len() as u32)?;
        if verify != data {
            let first = verify
                .iter()
                .zip(data.iter())
                .position(|(written, expected)| written != expected)
                .unwrap_or(0);
            return Err(format!(
                "flash verify failed at 0x{:08X}: wrote 0x{:02X}, read back 0x{:02X}",
                address + first as u32,
                data[first],
                verify[first]
            ));
        }
        Ok(())
    }

    fn flash_unlock(&mut self) -> Result<(), String> {
        self.write_flash_register(flash_reg::KEYR, FLASH_KEY1)?;
        self.write_flash_register(flash_reg::KEYR, FLASH_KEY2)?;
        if self.read_flash_register(flash_reg::CR)? & FLASH_CR_LOCK != 0 {
            return Err("the flash controller did not accept the unlock sequence".to_string());
        }
        self.write_flash_register(flash_reg::SR, FLASH_SR_EOP | FLASH_SR_WRPRTERR)?;
        Ok(())
    }

    fn flash_lock(&mut self) {
        let _ = self.write_flash_register(flash_reg::CR, FLASH_CR_LOCK);
    }

    fn read_flash_register(&self, offset: u32) -> Result<u32, String> {
        self.cpu
            .read_memory(map::flash::BASE + offset, AccessWidth::Word)
            .map_err(|error| error.to_string())
    }

    fn write_flash_register(&mut self, offset: u32, value: u32) -> Result<(), String> {
        self.cpu
            .write_memory(map::flash::BASE + offset, AccessWidth::Word, value)
            .map_err(|error| error.to_string())
    }

    /// Places a breakpoint. Software and hardware breakpoints behave the same
    /// here: the core compares the program counter against the list, so no
    /// instruction has to be patched in flash.
    pub fn insert_breakpoint(&mut self, kind: PointKind, address: u32, _size: u32) -> bool {
        let address = address & !1;
        let hardware = kind == PointKind::Hardware;
        if !self
            .breakpoints
            .iter()
            .any(|point| point.address == address)
        {
            self.breakpoints.push(Breakpoint { address, hardware });
            self.cpu.add_breakpoint(address);
        }
        true
    }

    /// Removes a breakpoint again.
    pub fn remove_breakpoint(&mut self, _kind: PointKind, address: u32, _size: u32) -> bool {
        let address = address & !1;
        self.breakpoints.retain(|point| point.address != address);
        self.cpu.remove_breakpoint(address);
        true
    }

    /// Adds a watchpoint and records the current contents to compare against.
    ///
    /// The core model has no comparators, so watchpoints are evaluated between
    /// instructions while the target runs.
    pub fn insert_watchpoint(&mut self, _kind: PointKind, address: u32, length: u32) -> bool {
        let length = length.max(1);
        self.watchpoints.push(Watchpoint { address, length });
        self.refresh_watch_values();
        true
    }

    /// Removes a watchpoint.
    pub fn remove_watchpoint(&mut self, address: u32, length: u32) -> bool {
        let length = length.max(1);
        self.watchpoints
            .retain(|point| point.address != address || point.length != length);
        self.refresh_watch_values();
        true
    }

    /// True while at least one watchpoint is armed.
    pub fn has_watchpoints(&self) -> bool {
        !self.watchpoints.is_empty()
    }

    fn refresh_watch_values(&mut self) {
        self.watch_values = self
            .watchpoints
            .iter()
            .map(|point| {
                self.read_memory(point.address, point.length)
                    .unwrap_or_default()
            })
            .collect();
    }

    fn watch_hit(&mut self) -> Option<u32> {
        for index in 0..self.watchpoints.len() {
            let point = self.watchpoints[index];
            let current = self
                .read_memory(point.address, point.length)
                .unwrap_or_default();
            if current != self.watch_values[index] {
                self.watch_values[index] = current;
                return Some(point.address);
            }
        }
        None
    }

    /// Clears the stop state so the core can run again.
    pub fn resume(&mut self) {
        self.cpu.resume();
        self.refresh_watch_values();
    }

    /// Executes a single instruction.
    pub fn step(&mut self) -> Result<Option<Stop>, String> {
        self.cpu
            .step_instruction()
            .map_err(|error| error.to_string())?;
        Ok(self
            .cpu
            .stop_reason()
            .and_then(|reason| self.describe(reason)))
    }

    /// Runs up to `budget` instructions; `None` means "still running".
    pub fn run(&mut self, budget: u64) -> Result<Option<Stop>, String> {
        if self.watchpoints.is_empty() {
            let reason = self.cpu.run(budget).map_err(|error| error.to_string())?;
            return Ok(self.describe(reason));
        }

        for _ in 0..budget {
            self.cpu
                .step_instruction()
                .map_err(|error| error.to_string())?;
            if let Some(address) = self.watch_hit() {
                self.cpu.halt();
                return Ok(Some(Stop::Watchpoint { address }));
            }
            if let Some(stop) = self
                .cpu
                .stop_reason()
                .and_then(|reason| self.describe(reason))
            {
                return Ok(Some(stop));
            }
        }
        Ok(None)
    }

    fn describe(&self, reason: StopReason) -> Option<Stop> {
        Some(match reason {
            StopReason::InstructionLimit => return None,
            StopReason::Breakpoint(address) => Stop::Breakpoint {
                address,
                hardware: self
                    .breakpoints
                    .iter()
                    .find(|point| point.address == address)
                    .is_some_and(|point| point.hardware),
            },
            StopReason::Break => Stop::Break,
            StopReason::Halted => Stop::Halted,
            StopReason::Reset => Stop::Reset,
            StopReason::Debug => Stop::Halt,
        })
    }

    /// Executes a `monitor` command and returns the text for the console.
    pub fn monitor(&mut self, command: &str) -> String {
        let mut words = command.split_whitespace();
        match words.next().unwrap_or("") {
            "" | "help" => "\
reset             reset the device and halt the core
halt              stop the core
regs              dump the core registers
info              instruction, clock and interrupt counters
peripherals       clock and state of every modelled block
firmware          description of the loaded ELF image
flash             flash geometry
"
            .to_string(),
            "reset" => match self.hard_reset() {
                Ok(()) => "the device was reset and the core is halted\n".to_string(),
                Err(message) => format!("{message}\n"),
            },
            "halt" => {
                self.cpu.halt();
                "the core is halted\n".to_string()
            }
            "regs" | "registers" => self.register_dump(),
            "info" | "status" => {
                let mut text = self.summary();
                text.push_str(&self.clock_summary());
                text
            }
            "peripherals" => self.peripheral_summary(),
            "firmware" => self.firmware_summary(),
            "flash" => format!(
                "flash: {} KiB at 0x{:08X}, {} byte pages\n",
                map::FLASH_SIZE / 1024,
                map::FLASH_BASE,
                description::FLASH_PAGE_SIZE
            ),
            other => format!("unknown monitor command `{other}`; try `monitor help`\n"),
        }
    }

    fn register_dump(&self) -> String {
        let mut text = String::new();
        for row in 0..4 {
            for column in 0..4 {
                let index = row * 4 + column;
                text.push_str(&format!(
                    "r{:<2} 0x{:08X}  ",
                    index,
                    self.read_register(index)
                ));
            }
            text.push('\n');
        }
        text.push_str(&format!(
            "sp  0x{:08X}  lr  0x{:08X}  pc  0x{:08X}  xpsr 0x{:08X}\n",
            self.read_register(13),
            self.read_register(14),
            self.read_register(15),
            self.read_register(description::XPSR_REGISTER)
        ));
        text
    }

    /// Counters that describe what the firmware did so far.
    pub fn summary(&self) -> String {
        let soc = self.cpu.soc();
        let soc = soc.borrow();
        format!(
            "instructions: {}\nsimulated time: {:.3} ms\nperipheral interrupts: {}\nresets: {}\ncore: {}\n",
            self.cpu.instructions(),
            soc.now_ns() as f64 / 1_000_000.0,
            self.cpu.interrupts(),
            soc.reset_count(),
            if self.cpu.halted() { "halted" } else { "running" }
        )
    }

    fn clock_summary(&self) -> String {
        let soc = self.cpu.soc();
        let soc = soc.borrow();
        let clocks = soc.clocks();
        format!(
            "SYSCLK: {:.3} MHz\nHCLK: {:.3} MHz\nPCLK1: {:.3} MHz\nPCLK2: {:.3} MHz\n",
            f64::from(clocks.sysclk_hz) / 1e6,
            f64::from(clocks.hclk_hz) / 1e6,
            f64::from(clocks.pclk1_hz) / 1e6,
            f64::from(clocks.pclk2_hz) / 1e6
        )
    }

    fn peripheral_summary(&self) -> String {
        let soc = self.cpu.soc();
        let soc = soc.borrow();
        let mut text = String::new();
        for status in soc.peripheral_status() {
            text.push_str(&format!(
                "{:<10} 0x{:08X} clock {}\n",
                status.name,
                status.base,
                gsim_stm32f1::status::hz(status.clock_hz)
            ));
        }
        text
    }

    fn firmware_summary(&self) -> String {
        match self.image() {
            Some(image) => format!(
                "{}: {} bytes in {} segments, entry 0x{:08X}\n",
                image.path().display(),
                image.total_size(),
                image.segments().len(),
                image.entry()
            ),
            None => "no ELF loaded; use GDB `load` to program the simulated flash\n".to_string(),
        }
    }
}
