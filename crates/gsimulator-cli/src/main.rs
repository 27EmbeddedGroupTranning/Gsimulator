//! Gsimulator command line front end.

mod demo;
mod flash;
mod gdb;
mod run;

use std::path::PathBuf;
use std::process::ExitCode;

const USAGE: &str = "\
Gsimulator - STM32F103 on-chip peripheral simulator (phase 1)

USAGE:
    gsimulator <COMMAND> [OPTIONS]

COMMANDS:
    run <firmware.elf>   Load and execute a real ARM Cortex-M3 ELF image
    gdb [firmware.elf]   Serve a virtual ST-Link so GDB can flash and debug
    flash <firmware.elf> Program an image into the simulated flash (and verify)
    demo                 Scripted tour of every phase 1 peripheral
    info                 Print the simulated memory map
    help                 Print this message
    version              Print the version

OPTIONS for `run`:
    -n, --max-instructions <N>   Instruction budget (default 5,000,000)
    -t, --max-microseconds <N>   Stop after N microseconds of simulated time
        --hse <HZ>               Crystal frequency on OSC_IN (default 8,000,000)
        --trace                  Trace peripheral events on stderr
        --registers              Dump the core registers when the run stops

OPTIONS for `gdb`:
    -p, --port <PORT>            TCP port of the debug server (default 3333)
        --bind <ADDR:PORT>       Address to listen on (default 127.0.0.1:3333)
        --elf <FIRMWARE.ELF>     Image loaded before the first GDB connection
        --hse <HZ>               Crystal frequency on OSC_IN (default 8,000,000)
        --no-uart                Do not print the firmware's UART output
    -v, --verbose                Log protocol activity on stderr

OPTIONS for `flash`:
        --hse <HZ>               Crystal frequency on OSC_IN (default 8,000,000)

UART output of the firmware is written to stdout; diagnostics go to stderr.";

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let Some(command) = args.next() else {
        eprintln!("{USAGE}");
        return ExitCode::from(2);
    };

    match command.as_str() {
        "run" => match parse_run(args) {
            Ok(options) => finish(run::run(options)),
            Err(message) => usage_error(&message),
        },
        "gdb" => match parse_gdb(args) {
            Ok(options) => finish(gdb::run(options)),
            Err(message) => usage_error(&message),
        },
        "flash" => match parse_flash(args) {
            Ok(options) => finish(flash::run(options)),
            Err(message) => usage_error(&message),
        },
        "demo" => finish(demo::run()),
        "info" | "map" => {
            print_memory_map();
            ExitCode::SUCCESS
        }
        "help" | "-h" | "--help" => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        "version" | "-V" | "--version" => {
            println!("gsimulator {}", env!("CARGO_PKG_VERSION"));
            ExitCode::SUCCESS
        }
        other => usage_error(&format!("unknown command `{other}`")),
    }
}

fn finish(result: Result<(), String>) -> ExitCode {
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::FAILURE
        }
    }
}

fn usage_error(message: &str) -> ExitCode {
    eprintln!("error: {message}");
    eprintln!();
    eprintln!("{USAGE}");
    ExitCode::from(2)
}

fn parse_run(args: impl Iterator<Item = String>) -> Result<run::RunOptions, String> {
    let mut options = run::RunOptions::default();
    let mut elf: Option<PathBuf> = None;
    let mut args = args.peekable();

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "-n" | "--max-instructions" => {
                options.max_instructions = parse_number(&mut args, "max-instructions")?;
            }
            "-t" | "--max-microseconds" => {
                options.max_microseconds = Some(parse_number(&mut args, "max-microseconds")?);
            }
            "--hse" => {
                options.hse_hz = parse_number(&mut args, "hse")? as u32;
            }
            "--trace" => options.trace = true,
            "--registers" => options.dump_registers = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                std::process::exit(0);
            }
            other if other.starts_with('-') => {
                return Err(format!("unknown option `{other}`"));
            }
            other => {
                if elf.is_some() {
                    return Err(format!("unexpected argument `{other}`"));
                }
                elf = Some(PathBuf::from(other));
            }
        }
    }

    options.elf = elf.ok_or_else(|| "`run` needs the path of an ELF file".to_string())?;
    Ok(options)
}

fn parse_gdb(args: impl Iterator<Item = String>) -> Result<gdb::GdbOptions, String> {
    let mut options = gdb::GdbOptions::default();
    let mut elf: Option<PathBuf> = None;
    let mut args = args.peekable();

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "-p" | "--port" => {
                let port = parse_number(&mut args, "port")?;
                if port > u64::from(u16::MAX) {
                    return Err(format!("`port` expects a TCP port, got `{port}`"));
                }
                options.bind.set_port(port as u16);
            }
            "--bind" => {
                let text = args
                    .next()
                    .ok_or_else(|| "`bind` needs an address".to_string())?;
                options.bind = text
                    .parse()
                    .map_err(|_| format!("`bind` expects ADDRESS:PORT, got `{text}`"))?;
            }
            "--elf" => {
                let text = args
                    .next()
                    .ok_or_else(|| "`elf` needs a path".to_string())?;
                if elf.is_some() {
                    return Err("the firmware image was given twice".to_string());
                }
                elf = Some(PathBuf::from(text));
            }
            "--hse" => options.hse_hz = parse_number(&mut args, "hse")? as u32,
            "--no-uart" => options.echo_uart = false,
            "-v" | "--verbose" => options.verbose = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                std::process::exit(0);
            }
            other if other.starts_with('-') => {
                return Err(format!("unknown option `{other}`"));
            }
            other => {
                if elf.is_some() {
                    return Err(format!("unexpected argument `{other}`"));
                }
                elf = Some(PathBuf::from(other));
            }
        }
    }

    options.elf = elf;
    Ok(options)
}

fn parse_flash(args: impl Iterator<Item = String>) -> Result<flash::FlashOptions, String> {
    let mut options = flash::FlashOptions::default();
    let mut elf: Option<PathBuf> = None;
    let mut args = args.peekable();

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--hse" => options.hse_hz = parse_number(&mut args, "hse")? as u32,
            "-h" | "--help" => {
                println!("{USAGE}");
                std::process::exit(0);
            }
            other if other.starts_with('-') => {
                return Err(format!("unknown option `{other}`"));
            }
            other => {
                if elf.is_some() {
                    return Err(format!("unexpected argument `{other}`"));
                }
                elf = Some(PathBuf::from(other));
            }
        }
    }

    options.elf = elf.ok_or_else(|| "`flash` needs the path of an ELF file".to_string())?;
    Ok(options)
}

fn parse_number(
    args: &mut std::iter::Peekable<impl Iterator<Item = String>>,
    name: &str,
) -> Result<u64, String> {
    let text = args
        .next()
        .ok_or_else(|| format!("`{name}` needs a value"))?;
    let trimmed = text.trim();
    let parsed = if let Some(hex) = trimmed.strip_prefix("0x") {
        u64::from_str_radix(hex, 16)
    } else {
        trimmed.replace('_', "").parse::<u64>()
    };
    parsed.map_err(|_| format!("`{name}` expects a number, got `{text}`"))
}

fn print_memory_map() {
    use gsim_stm32f1::{bitband, map};

    println!("STM32F103 memory map (as simulated)");
    println!("{:<26} {:<12} size", "region", "base");
    let regions: [(&str, u32, u32); 28] = [
        ("flash", map::FLASH_BASE, map::FLASH_SIZE),
        ("flash alias", 0x0000_0000, map::FLASH_SIZE),
        ("sram", map::SRAM_BASE, map::SRAM_SIZE),
        ("TIM2", map::tim::BASE_2, 0x400),
        ("TIM3", map::tim::BASE_3, 0x400),
        ("TIM4", map::tim::BASE_4, 0x400),
        ("RTC", map::rtc::BASE, 0x400),
        ("WWDG", map::wwdg::BASE, 0x400),
        ("IWDG", map::iwdg::BASE, 0x400),
        ("SPI2", map::spi::BASE_2, 0x400),
        ("USART2", map::usart::BASE_2, 0x400),
        ("USART3", map::usart::BASE_3, 0x400),
        ("I2C1", map::i2c::BASE_1, 0x400),
        ("I2C2", map::i2c::BASE_2, 0x400),
        ("BKP", map::bkp::BASE, 0x400),
        ("PWR", map::pwr::BASE, 0x400),
        ("AFIO", map::afio::BASE, 0x400),
        ("EXTI", map::exti::BASE, 0x400),
        ("GPIOA..E", map::gpio::BASE_A, 0x1400),
        ("ADC1", map::adc::BASE_1, 0x400),
        ("ADC2", map::adc::BASE_2, 0x400),
        ("TIM1", map::tim::BASE_1, 0x400),
        ("SPI1", map::spi::BASE_1, 0x400),
        ("USART1", map::usart::BASE_1, 0x400),
        ("DMA1", map::dma1::BASE, 0x400),
        ("DMA2", map::dma2::BASE, 0x400),
        ("CAN1", map::can::BASE_1, 0x400),
        ("RCC", map::rcc::BASE, 0x400),
    ];
    for (name, base, size) in regions {
        println!("{name:<26} 0x{base:08X}   {size} bytes");
    }
    println!(
        "{:<26} 0x{:08X}   {} bytes",
        "FLASH interface",
        map::flash::BASE,
        0x400
    );
    println!("{:<26} 0x{:08X}   {} bytes", "CRC", map::crc::BASE, 0x400);
    println!(
        "{:<26} 0x{:08X}   {} bytes",
        "DBGMCU",
        map::dbgmcu::BASE,
        0x400
    );
    println!(
        "{:<26} 0x{:08X}   {} bytes",
        "SCB / SysTick / NVIC", 0xE000_E000u32, 0x1000
    );
    println!(
        "{:<26} 0x{:08X}   {} bytes (bit-band alias)",
        "SRAM alias",
        bitband::SRAM_ALIAS,
        bitband::ALIAS_SIZE
    );
    println!(
        "{:<26} 0x{:08X}   {} bytes (bit-band alias)",
        "peripheral alias",
        bitband::PERIPHERAL_ALIAS,
        bitband::ALIAS_SIZE
    );
    println!();
    println!(
        "Off-chip models (gsim-devices): MAX7219 8x8 LED matrix on SPI1 (PA4 LOAD) \
         and a CAN transceiver with a shared virtual bus on CAN1."
    );
    println!("Not modelled: CAN2/USB, external I2C devices, bit level CAN arbitration.");
}
