//! ARM Cortex-M3 execution front end.
//!
//! The instruction set is provided by [`armagnac`], a pure Rust Thumb / ARMv7-M
//! emulator. This module owns the integration:
//!
//! * every memory window the core can reach is mapped onto the [`Soc`] bus,
//! * one executed instruction costs one core clock cycle, and the SoC is
//!   advanced by the matching amount of simulated time,
//! * interrupts are arbitrated with the NVIC state the firmware programmed
//!   (`ISER`/`IPR`/`AIRCR.PRIGROUP`), and the winner is handed to the core with
//!   [`Processor::request_interrupt`], which performs the ARMv7-M exception
//!   entry (stacking, `EXC_RETURN`) and return sequence.

use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use armagnac::core::{
    ArmVersion, Config, Emulator, Event, Irq as ArmIrq, Processor, RunError, RunOptions,
};
use armagnac::decoder::Lut16AndGrouped32InstructionDecoder;
use armagnac::registers::CoreRegisters;
use gsim_core::{AccessWidth, BusError, Irq, MemoryBus};
use gsim_stm32f1::{bitband, map, Soc};

use crate::elf::{ElfError, ElfImage};
use crate::interface::SocInterface;

/// Number of external interrupt lines of the STM32F103.
pub const EXTERNAL_IRQ_COUNT: u16 = 60;

const AIRCR: u32 = 0xE000_ED0C;
const SHPR1: u32 = 0xE000_ED18;
const SHPR2: u32 = 0xE000_ED1C;
const SHPR3: u32 = 0xE000_ED20;
const NVIC_ISER: u32 = 0xE000_E100;
const NVIC_ICER: u32 = 0xE000_E180;
const NVIC_IPR: u32 = 0xE000_E400;

/// Words of 32 interrupt lines in the NVIC enable/disable sets.
const NVIC_WORDS: usize = EXTERNAL_IRQ_COUNT as usize / 32 + 1;

/// Scheduler steps the front end observes the NVIC one instruction at a time
/// after it saw the enable set change.
///
/// `HAL_NVIC_EnableIRQ()` stores a single bit into `NVIC_ISERn`, and the core
/// model keeps the *last value written* instead of the sticky set hardware
/// implements. Sampling the register after every batch and then watching it
/// instruction by instruction for a while reconstructs the same mask a real
/// NVIC keeps, even when a firmware enables several lines back to back.
const NVIC_CLOSE_LOOK_STEPS: u32 = 512;

/// Size of the peripheral window mapped into the core address space.
const PERIPHERAL_WINDOW: u32 = 0x0004_0000;

/// Instructions executed between two peripheral clock updates.
///
/// Peripherals are advanced with the exact number of elapsed cycles, but
/// servicing them once per instruction would dominate the run time. The
/// default keeps interrupt latency below half a microsecond at 72 MHz.
pub const DEFAULT_BATCH: u32 = 32;

/// Reason the core stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// The instruction budget ran out.
    InstructionLimit,
    /// A breakpoint hook was reached.
    Breakpoint(u32),
    /// The firmware executed a `BKPT` instruction.
    Break,
    /// The firmware halted the core (`WFI` with no wake-up source left).
    Halted,
    /// The device was reset while running.
    Reset,
    /// A host tool (the debug server) halted the core.
    Debug,
}

/// Errors raised by the CPU front end.
#[derive(Debug)]
pub enum CpuError {
    Elf(ElfError),
    /// The core could not map a memory window.
    Mapping {
        base: u32,
    },
    /// The core raised an emulation error.
    Emulation {
        pc: u32,
        error: RunError,
    },
    /// A register read/write through the SoC bus failed.
    Bus(BusError),
    /// No valid vector table was found in flash.
    NoVectorTable,
}

impl std::fmt::Display for CpuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CpuError::Elf(error) => write!(f, "{error}"),
            CpuError::Mapping { base } => {
                write!(f, "cannot map the core memory window at 0x{base:08X}")
            }
            CpuError::Emulation { pc, error } => {
                write!(f, "emulation error at PC 0x{pc:08X}: {error:?}")
            }
            CpuError::Bus(error) => write!(f, "{error}"),
            CpuError::NoVectorTable => write!(f, "no valid vector table in flash"),
        }
    }
}

impl std::error::Error for CpuError {}

impl From<ElfError> for CpuError {
    fn from(error: ElfError) -> Self {
        CpuError::Elf(error)
    }
}

impl From<BusError> for CpuError {
    fn from(error: BusError) -> Self {
        CpuError::Bus(error)
    }
}

/// Cortex-M3 core attached to a [`Soc`].
pub struct Cpu {
    proc: Processor,
    soc: Rc<RefCell<Soc>>,
    instructions: u64,
    ns_remainder: u64,
    halted: bool,
    stop_reason: Option<StopReason>,
    breakpoints: Vec<u32>,
    skip_breakpoint: Option<u32>,
    last_reset_count: u32,
    interrupt_count: u64,
    batch: u32,
    /// Sticky copy of the NVIC enable set (see [`NVIC_CLOSE_LOOK_STEPS`]).
    nvic_enabled: [u32; NVIC_WORDS],
    /// Last `ISER`/`ICER` values observed, per 32 line word.
    nvic_iser: [u32; NVIC_WORDS],
    nvic_icer: [u32; NVIC_WORDS],
    /// Scheduler steps left with a one instruction batch.
    close_look: u32,
}

impl Cpu {
    /// Creates a core with the STM32F103 memory map already attached.
    pub fn new(soc: Rc<RefCell<Soc>>) -> Result<Self, CpuError> {
        let mut proc =
            Processor::new(Config::v7m().external_exceptions(EXTERNAL_IRQ_COUNT as usize));
        // The look-up table decoder is noticeably faster for long runs.
        proc.instruction_decoder =
            Box::new(Lut16AndGrouped32InstructionDecoder::new(ArmVersion::V7M));

        let windows = [
            (map::FLASH_BASE, map::FLASH_BASE, map::FLASH_SIZE),
            // Boot alias: the F103 mirrors flash at address 0 so that the
            // vector table works before VTOR is programmed.
            (0x0000_0000, map::FLASH_BASE, map::FLASH_SIZE),
            (map::SRAM_BASE, map::SRAM_BASE, map::SRAM_SIZE),
            (map::APB1_BASE, map::APB1_BASE, PERIPHERAL_WINDOW),
            (map::DBGMCU_BASE, map::DBGMCU_BASE, 0x1000),
            // Cortex-M3 bit-band aliases; the HAL writes single bits through
            // them (`RCC_CR_PLLON_BB`, `GPIOx_ODR_BB`, ...).
            (
                bitband::SRAM_ALIAS,
                bitband::SRAM_ALIAS,
                bitband::ALIAS_SIZE,
            ),
            (
                bitband::PERIPHERAL_ALIAS,
                bitband::PERIPHERAL_ALIAS,
                bitband::ALIAS_SIZE,
            ),
        ];

        for (mapped_base, soc_base, size) in windows {
            let interface = SocInterface::new(Rc::clone(&soc), soc_base, size);
            proc.map_iface(mapped_base, Rc::new(RefCell::new(interface)))
                .map_err(|_| CpuError::Mapping { base: mapped_base })?;
        }

        // The core owns the Cortex-M system control space, so the SoC must not
        // run a second SysTick in parallel.
        soc.borrow_mut().set_external_systick(true);
        let last_reset_count = soc.borrow().reset_count();

        Ok(Cpu {
            proc,
            soc,
            instructions: 0,
            ns_remainder: 0,
            halted: false,
            stop_reason: None,
            breakpoints: Vec::new(),
            skip_breakpoint: None,
            last_reset_count,
            interrupt_count: 0,
            batch: DEFAULT_BATCH,
            nvic_enabled: [0; NVIC_WORDS],
            nvic_iser: [0; NVIC_WORDS],
            nvic_icer: [0; NVIC_WORDS],
            close_look: 0,
        })
    }

    /// Number of instructions executed between two peripheral updates.
    pub fn batch(&self) -> u32 {
        self.batch
    }

    /// Changes the instruction batch size; `1` gives cycle exact interrupt
    /// latency at the cost of speed.
    pub fn set_batch(&mut self, batch: u32) {
        self.batch = batch.max(1);
    }

    /// Shared handle to the simulated device.
    pub fn soc(&self) -> Rc<RefCell<Soc>> {
        Rc::clone(&self.soc)
    }

    pub fn pc(&self) -> u32 {
        self.proc.pc()
    }

    pub fn sp(&self) -> u32 {
        self.proc.sp()
    }

    pub fn lr(&self) -> u32 {
        self.proc.lr()
    }

    /// General purpose register `index` (0..=12) or `sp`/`lr`/`pc`.
    pub fn register(&self, index: u16) -> Option<u32> {
        Some(match index {
            0 => self.proc.registers.r0,
            1 => self.proc.registers.r1,
            2 => self.proc.registers.r2,
            3 => self.proc.registers.r3,
            4 => self.proc.registers.r4,
            5 => self.proc.registers.r5,
            6 => self.proc.registers.r6,
            7 => self.proc.registers.r7,
            8 => self.proc.registers.r8,
            9 => self.proc.registers.r9,
            10 => self.proc.registers.r10,
            11 => self.proc.registers.r11,
            12 => self.proc.registers.r12,
            13 => self.proc.sp(),
            14 => self.proc.lr(),
            15 => self.proc.pc(),
            _ => return None,
        })
    }

    /// Writes a core register from the host side, the way a debugger does.
    ///
    /// Returns `false` for indices outside the register file (`0..=15`).
    pub fn set_register(&mut self, index: u16, value: u32) -> bool {
        match index {
            0 => self.proc.registers.r0 = value,
            1 => self.proc.registers.r1 = value,
            2 => self.proc.registers.r2 = value,
            3 => self.proc.registers.r3 = value,
            4 => self.proc.registers.r4 = value,
            5 => self.proc.registers.r5 = value,
            6 => self.proc.registers.r6 = value,
            7 => self.proc.registers.r7 = value,
            8 => self.proc.registers.r8 = value,
            9 => self.proc.registers.r9 = value,
            10 => self.proc.registers.r10 = value,
            11 => self.proc.registers.r11 = value,
            12 => self.proc.registers.r12 = value,
            13 => self.proc.set_sp(value),
            14 => self.proc.set_lr(value),
            15 => self.proc.set_pc(value),
            _ => return false,
        }
        true
    }

    /// Value of `xPSR`: condition flags, exception number and execution state.
    pub fn xpsr(&self) -> u32 {
        self.proc.registers.psr.get()
    }

    /// Overwrites `xPSR`; the core model masks the reserved bits.
    pub fn set_xpsr(&mut self, value: u32) {
        self.proc.registers.psr.set(value);
    }

    /// Stops the core before its next instruction, as a halt request from a
    /// debugger does. The reason is reported as [`StopReason::Debug`].
    pub fn halt(&mut self) {
        self.halted = true;
        self.stop_reason = Some(StopReason::Debug);
        self.skip_breakpoint = None;
    }

    /// Executes exactly one instruction, independent of the batch size.
    ///
    /// A breakpoint sitting on the current `pc` does not stop the step: the
    /// caller asked for that instruction to execute.
    pub fn step_instruction(&mut self) -> Result<(), CpuError> {
        let pc = self.proc.pc();
        self.halted = false;
        self.stop_reason = None;
        self.skip_breakpoint = Some(pc);
        self.step_with_budget(1)
    }

    /// Number of instructions executed since the last reset.
    pub fn instructions(&self) -> u64 {
        self.instructions
    }

    /// Number of peripheral interrupt requests handed to the core.
    ///
    /// SysTick is driven inside the core model and is therefore not included.
    pub fn interrupts(&self) -> u64 {
        self.interrupt_count
    }

    pub fn halted(&self) -> bool {
        self.halted
    }

    pub fn stop_reason(&self) -> Option<StopReason> {
        self.stop_reason
    }

    /// Resumes a halted core. When it stopped on a breakpoint, that breakpoint
    /// is skipped once so the instruction below it executes.
    pub fn resume(&mut self) {
        if let Some(StopReason::Breakpoint(address)) = self.stop_reason {
            self.skip_breakpoint = Some(address);
        }
        self.halted = false;
        self.stop_reason = None;
    }

    /// Sets a code address at which the core stops before executing.
    ///
    /// Breakpoints are checked per instruction in the run loop, which keeps them
    /// precise; without breakpoints the core runs in batches.
    pub fn add_breakpoint(&mut self, address: u32) {
        if !self.breakpoints.contains(&address) {
            self.breakpoints.push(address);
        }
    }

    /// Removes a breakpoint again.
    pub fn remove_breakpoint(&mut self, address: u32) {
        self.breakpoints.retain(|candidate| *candidate != address);
    }

    /// Reads a register of the core's system control space (`0xE000_E000`).
    ///
    /// Firmware reaches the same registers through normal memory accesses; this
    /// helper is for tools and tests that drive the core from the host side.
    pub fn read_core_register(&mut self, address: u32) -> Result<u32, CpuError> {
        self.read_u32(address)
    }

    /// Writes a register of the core's system control space (`0xE000_E000`).
    pub fn write_core_register(&mut self, address: u32, value: u32) -> Result<(), CpuError> {
        self.proc
            .write_u32_aligned(address, value)
            .map_err(|error| CpuError::Emulation { pc: address, error })
    }

    /// Loads an image into the device and resets the core from its vector table.
    pub fn load(&mut self, image: &ElfImage) -> Result<(), CpuError> {
        image.load_into(&mut self.soc.borrow_mut())?;
        self.reset()
    }

    /// Loads an ELF file and resets the core.
    pub fn load_file(&mut self, path: impl AsRef<Path>) -> Result<ElfImage, CpuError> {
        let image = ElfImage::open(path)?;
        self.load(&image)?;
        Ok(image)
    }

    /// Resets the core to the reset handler described by the vector table.
    pub fn reset(&mut self) -> Result<(), CpuError> {
        let (sp, reset) = {
            let soc = self.soc.borrow();
            let sp = soc.memory.read(map::FLASH_BASE, AccessWidth::Word)?;
            let reset = soc.memory.read(map::FLASH_BASE + 4, AccessWidth::Word)?;
            (sp, reset)
        };
        if reset == 0 || reset == 0xFFFF_FFFF {
            return Err(CpuError::NoVectorTable);
        }
        self.proc.registers = CoreRegisters::new();
        self.proc.set_sp(sp);
        self.proc.set_pc(reset & !1);
        self.instructions = 0;
        self.ns_remainder = 0;
        self.halted = false;
        self.stop_reason = None;
        self.interrupt_count = 0;
        Ok(())
    }

    /// Executes up to `budget` instructions and advances the device by the
    /// number of cycles that were consumed.
    pub fn step(&mut self) -> Result<(), CpuError> {
        self.step_with_budget(self.batch)
    }

    /// Executes at most `budget` instructions.
    ///
    /// Without breakpoints the core runs the whole batch in one call, which is
    /// much faster; with breakpoints it runs one instruction at a time so the
    /// PC can be compared against the breakpoint list.
    fn step_with_budget(&mut self, budget: u32) -> Result<(), CpuError> {
        if self.halted || budget == 0 {
            return Ok(());
        }
        self.service_interrupts()?;

        let start = self.proc.cycles;
        if self.breakpoints.is_empty() {
            let mut remaining = budget as usize;
            while remaining > 0 && !self.halted {
                match self.proc.run(RunOptions::new().gas(remaining)) {
                    Ok(Some(event)) => self.handle_event(event)?,
                    Ok(None) => break,
                    Err(error) => {
                        return Err(CpuError::Emulation {
                            pc: self.proc.pc(),
                            error,
                        })
                    }
                }
                let executed = (self.proc.cycles - start) as u32;
                if executed == 0 {
                    break;
                }
                remaining = (budget - executed) as usize;
            }
        } else {
            for _ in 0..budget {
                if self.halted {
                    break;
                }
                let pc = self.proc.pc();
                if self.breakpoints.contains(&pc) {
                    if self.skip_breakpoint == Some(pc) {
                        self.skip_breakpoint = None;
                    } else {
                        self.halted = true;
                        self.stop_reason = Some(StopReason::Breakpoint(pc));
                        break;
                    }
                }
                match self.proc.run(RunOptions::new().gas(1)) {
                    Ok(Some(event)) => self.handle_event(event)?,
                    Ok(None) => {}
                    Err(error) => return Err(CpuError::Emulation { pc, error }),
                }
            }
        }

        let cycles = self.proc.cycles - start;
        self.instructions += cycles;
        self.advance_time(cycles);
        self.sync_device_resets();
        Ok(())
    }

    /// Runs until `max_instructions` instructions were executed, stopping early
    /// on a breakpoint or when the firmware halts.
    pub fn run(&mut self, max_instructions: u64) -> Result<StopReason, CpuError> {
        let limit = self.instructions + max_instructions;
        while self.instructions < limit && !self.halted {
            let remaining = (limit - self.instructions).min(u64::from(self.scheduler_batch())) as u32;
            self.step_with_budget(remaining)?;
        }
        Ok(self.stop_reason.unwrap_or(if self.halted {
            StopReason::Halted
        } else {
            StopReason::InstructionLimit
        }))
    }

    /// Runs until `nanoseconds` of simulated time elapsed.
    pub fn run_for_ns(
        &mut self,
        nanoseconds: u64,
        max_instructions: u64,
    ) -> Result<StopReason, CpuError> {
        let deadline = self.soc.borrow().now_ns() + nanoseconds;
        let limit = self.instructions + max_instructions;
        while self.instructions < limit && !self.halted {
            if self.soc.borrow().now_ns() >= deadline {
                return Ok(StopReason::InstructionLimit);
            }
            let remaining = (limit - self.instructions).min(u64::from(self.scheduler_batch())) as u32;
            self.step_with_budget(remaining)?;
        }
        Ok(self.stop_reason.unwrap_or(if self.halted {
            StopReason::Halted
        } else {
            StopReason::InstructionLimit
        }))
    }

    /// Batch used by the run loop, temporarily one instruction while the NVIC
    /// enable set is changing.
    fn scheduler_batch(&self) -> u32 {
        if self.close_look > 0 {
            1
        } else {
            self.batch
        }
    }

    fn handle_event(&mut self, event: Event) -> Result<(), CpuError> {
        match event {
            Event::Hook { address } => {
                self.halted = true;
                self.stop_reason = Some(StopReason::Breakpoint(address));
            }
            Event::Break(code) => {
                self.halted = true;
                self.stop_reason = Some(StopReason::Break);
                let _ = code;
            }
            Event::Reset => {
                self.soc.borrow_mut().system_reset(0);
                self.reset()?;
                self.stop_reason = Some(StopReason::Reset);
            }
            Event::DebugHint(_) | Event::Instruction { .. } => {}
        }
        Ok(())
    }

    /// Advances the device by `cycles` core clock cycles of simulated time.
    fn advance_time(&mut self, cycles: u64) {
        if cycles == 0 {
            return;
        }
        let hclk = u64::from(self.soc.borrow().clocks().hclk_hz.max(1));
        self.ns_remainder += cycles * 1_000_000_000;
        let nanoseconds = self.ns_remainder / hclk;
        self.ns_remainder %= hclk;
        if nanoseconds > 0 {
            self.soc.borrow_mut().advance_ns(nanoseconds);
        }
    }

    /// Mirrors device resets (watchdogs) into the core.
    fn sync_device_resets(&mut self) {
        let count = self.soc.borrow().reset_count();
        if count != self.last_reset_count {
            self.last_reset_count = count;
            let _ = self.reset();
            self.stop_reason = Some(StopReason::Reset);
        }
    }

    fn read_u32(&mut self, address: u32) -> Result<u32, CpuError> {
        self.proc
            .read_u32_aligned(address)
            .map_err(|error| CpuError::Emulation { pc: address, error })
    }

    fn irq_enabled(&mut self, number: u16) -> Result<bool, CpuError> {
        let index = u32::from(number);
        let word = (index / 32) as usize;
        let bit = 1 << (index % 32);
        Ok(self
            .nvic_enabled
            .get(word)
            .is_some_and(|value| value & bit != 0))
    }

    /// Rebuilds the sticky NVIC enable set from the core's `ISER`/`ICER`
    /// registers.
    ///
    /// Real hardware keeps every bit written to `ISER` set until software
    /// writes the matching `ICER` bit. The core model stores the last written
    /// value instead, so a firmware that enables four lines in a row would only
    /// leave one visible. Sampling here and switching to a one instruction
    /// batch while the set is changing recovers the same mask.
    fn sync_nvic(&mut self) {
        let mut changed = false;
        for word in 0..NVIC_WORDS {
            let offset = word as u32 * 4;
            let iser = self.read_u32(NVIC_ISER + offset).unwrap_or(0);
            if iser != self.nvic_iser[word] {
                self.nvic_iser[word] = iser;
                self.nvic_enabled[word] |= iser;
                changed = true;
            }
            let icer = self.read_u32(NVIC_ICER + offset).unwrap_or(0);
            if icer != self.nvic_icer[word] {
                self.nvic_icer[word] = icer;
                self.nvic_enabled[word] &= !icer;
                changed = true;
            }
        }
        if changed {
            self.close_look = NVIC_CLOSE_LOOK_STEPS;
        } else {
            self.close_look = self.close_look.saturating_sub(1);
        }
    }

    fn irq_priority(&mut self, number: u16) -> Result<u8, CpuError> {
        let index = u32::from(number);
        let value = self.read_u32(NVIC_IPR + (index & !3))?;
        Ok(((value >> ((index & 3) * 8)) & 0xFF) as u8)
    }

    fn system_priority(&mut self, exception: u16) -> Result<u8, CpuError> {
        let (register, shift) = match exception {
            4..=7 => (SHPR1, (exception - 4) * 8),
            8..=11 => (SHPR2, (exception - 8) * 8),
            12..=15 => (SHPR3, (exception - 12) * 8),
            _ => return Ok(0),
        };
        let value = self.read_u32(register)?;
        Ok(((value >> shift) & 0xFF) as u8)
    }

    /// Pre-emption priority of a Cortex-M exception number.
    fn preempt_of(&mut self, exception: u16, prigroup: u8) -> Result<u16, CpuError> {
        let priority = match exception {
            // NMI and HardFault are special: they always win.
            2 => return Ok(0),
            3 => return Ok(1),
            4..=15 => self.system_priority(exception)?,
            _ => self.irq_priority(exception - 16)?,
        };
        Ok(u16::from(priority) >> (u32::from(prigroup) + 1))
    }

    /// Requests the highest priority pending interrupt from the core.
    fn service_interrupts(&mut self) -> Result<(), CpuError> {
        self.sync_nvic();
        if self.proc.registers.primask.pm() {
            return Ok(());
        }
        if self.soc.borrow().nvic.pending_count() == 0 {
            return Ok(());
        }

        // IPSR is 0 in thread mode and holds the exception number in a handler.
        let active = self.proc.registers.psr.exception_number();
        let prigroup = ((self.read_u32(AIRCR)? >> 8) & 0x7) as u8;
        let current = if active == 0 {
            0x100
        } else {
            self.preempt_of(active, prigroup)?
        };

        let pending: Vec<u16> = self
            .soc
            .borrow()
            .nvic
            .pending_irqs()
            .map(|irq| irq.number())
            .collect();

        let mut best: Option<(u16, u16)> = None;
        for number in pending {
            if number >= EXTERNAL_IRQ_COUNT {
                continue;
            }
            if !self.irq_enabled(number)? {
                continue;
            }
            let preempt = self.preempt_of(16 + number, prigroup)?;
            if preempt < current
                && best.map_or(true, |(best_preempt, best_number)| {
                    (preempt, number) < (best_preempt, best_number)
                })
            {
                best = Some((preempt, number));
            }
        }

        if let Some((_, number)) = best {
            self.interrupt_count += 1;
            self.proc.request_interrupt(ArmIrq::External(number));
        }
        Ok(())
    }

    /// Reads a byte of device memory through the SoC bus.
    pub fn read_memory(&self, address: u32, width: AccessWidth) -> Result<u32, BusError> {
        self.soc.borrow_mut().read(address, width)
    }

    /// Writes a byte of device memory through the SoC bus.
    pub fn write_memory(
        &self,
        address: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        self.soc.borrow_mut().write(address, width, value)
    }

    /// Injects an interrupt request into the NVIC pending bitmap, as a test
    /// harness or a debugger would.
    pub fn pend_interrupt(&mut self, irq: Irq) {
        self.soc.borrow_mut().nvic.set_pending(irq, true);
    }

    /// Records the device reset counter without restarting the core.
    ///
    /// A host tool that resets the device itself (the debug server behind
    /// `monitor reset`) calls this so the reset is not reported a second time
    /// as a stop reason on the next step.
    pub fn adopt_device_reset(&mut self) {
        self.last_reset_count = self.soc.borrow().reset_count();
    }
}
