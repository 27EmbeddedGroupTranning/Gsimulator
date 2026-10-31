//! `gsimulator run` – execute a real ARM Cortex-M3 ELF firmware image.

use std::cell::RefCell;
use std::io::Write;
use std::path::PathBuf;
use std::rc::Rc;

use gsim_core::Event;
use gsim_cpu::{Cpu, CpuError, ElfImage, RunError, StopReason};
use gsim_stm32f1::Soc;

/// Options of the `run` sub command.
pub struct RunOptions {
    pub elf: PathBuf,
    pub hse_hz: u32,
    pub max_instructions: u64,
    pub max_microseconds: Option<u64>,
    pub trace: bool,
    pub dump_registers: bool,
}

impl Default for RunOptions {
    fn default() -> Self {
        RunOptions {
            elf: PathBuf::new(),
            hse_hz: 8_000_000,
            max_instructions: 5_000_000,
            max_microseconds: None,
            trace: false,
            dump_registers: false,
        }
    }
}

/// Loads the image, runs it and reports what happened.
///
/// UART output goes to stdout so it can be piped; every diagnostic goes to
/// stderr.
pub fn run(options: RunOptions) -> Result<(), String> {
    let image = ElfImage::open(&options.elf).map_err(|error| error.to_string())?;

    let soc = Rc::new(RefCell::new(Soc::with_hse(options.hse_hz)));
    let trace = options.trace;
    soc.borrow_mut()
        .add_event_sink(Box::new(move |event: &Event| match event {
            Event::UartTx { unit: 0, byte } => {
                let mut stdout = std::io::stdout();
                let _ = stdout.write_all(&[*byte]);
                let _ = stdout.flush();
            }
            Event::Irq { .. } | Event::PinOutput { .. } if trace => {
                eprintln!("[trace] {event:?}");
            }
            Event::Note { .. } | Event::SpiTransfer { .. } | Event::CanFrame { .. } if trace => {
                eprintln!("[trace] {event:?}");
            }
            _ => {}
        }));

    let mut cpu = Cpu::new(Rc::clone(&soc)).map_err(|error| error.to_string())?;
    cpu.load(&image).map_err(|error| error.to_string())?;

    eprintln!(
        "loaded {} ({} bytes, entry 0x{:08X})",
        options.elf.display(),
        image.total_size(),
        image.entry()
    );

    let reason = match options.max_microseconds {
        Some(microseconds) => cpu
            .run_for_ns(microseconds * 1_000, options.max_instructions)
            .map_err(|error| diagnose(&error, &image)),
        None => cpu
            .run(options.max_instructions)
            .map_err(|error| diagnose(&error, &image)),
    }?;

    if options.dump_registers {
        eprintln!(
            "r0=0x{:08X} r1=0x{:08X} r2=0x{:08X} r3=0x{:08X}",
            cpu.register(0).unwrap_or(0),
            cpu.register(1).unwrap_or(0),
            cpu.register(2).unwrap_or(0),
            cpu.register(3).unwrap_or(0)
        );
        eprintln!(
            "r12=0x{:08X} sp=0x{:08X} lr=0x{:08X} pc=0x{:08X}",
            cpu.register(12).unwrap_or(0),
            cpu.sp(),
            cpu.lr(),
            cpu.pc()
        );
        if let Some(symbol) = image.function_at(cpu.pc()) {
            eprintln!("at {}", symbol.name);
        }
    }

    let stats = soc.borrow().stats();
    eprintln!(
        "stopped: {} | {} instructions | {:.3} ms simulated | {} peripheral IRQs | {} bus accesses",
        describe(&reason),
        cpu.instructions(),
        soc.borrow().now_ns() as f64 / 1_000_000.0,
        cpu.interrupts(),
        stats.reads + stats.writes
    );

    match reason {
        StopReason::Breakpoint(address) => {
            let name = image
                .function_at(address)
                .map(|symbol| symbol.name.clone())
                .unwrap_or_else(|| "<unknown>".to_string());
            eprintln!("breakpoint hit at 0x{address:08X} ({name})");
        }
        StopReason::Reset => eprintln!("the device was reset while running"),
        _ => {}
    }
    Ok(())
}

fn describe(reason: &StopReason) -> String {
    match reason {
        StopReason::InstructionLimit => "instruction limit reached".to_string(),
        StopReason::Breakpoint(address) => format!("breakpoint at 0x{address:08X}"),
        StopReason::Break => "BKPT instruction".to_string(),
        StopReason::Halted => "core halted".to_string(),
        StopReason::Reset => "reset".to_string(),
        StopReason::Debug => "halted by the debugger".to_string(),
    }
}

/// Turns an emulation failure into a message that names the offending address
/// and, when the symbol table knows it, the firmware function that caused it.
fn diagnose(error: &CpuError, image: &ElfImage) -> String {
    let (pc, address, access) = match error {
        CpuError::Emulation {
            pc,
            error: RunError::MemRead { address, .. },
        } => (*pc, *address, "read"),
        CpuError::Emulation {
            pc,
            error: RunError::MemWrite { address, .. },
        } => (*pc, *address, "write"),
        _ => return error.to_string(),
    };
    let function = image
        .function_at(pc)
        .map(|symbol| format!(" ({})", symbol.name))
        .unwrap_or_default();
    format!(
        "bus error on the {access} of 0x{address:08X}: {}\
         \n            while executing 0x{pc:08X}{function}",
        describe_missing(address)
    )
}

/// Explains which part of the address map is missing, if it is a known gap.
fn describe_missing(address: u32) -> &'static str {
    if (0x4000_0000..=0x4000_7FFF).contains(&address) {
        "unmapped APB1 peripheral"
    } else if (0x4001_0000..=0x4001_7FFF).contains(&address) {
        "unmapped APB2 peripheral"
    } else if (0x4002_0000..=0x4003_FFFF).contains(&address) {
        "unmapped AHB peripheral"
    } else {
        "unmapped address"
    }
}
