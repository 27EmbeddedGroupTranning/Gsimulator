//! `gsimulator flash` – program an ELF image into the simulated flash.
//!
//! This is the drop-in replacement for `openocd -f flash.cfg` (`program
//! build/Debug/app.elf verify`): the image is erased page by page, programmed
//! half word by half word through the FLASH controller and read back.

use std::path::PathBuf;

use gsim_cpu::ElfImage;
use gsim_gdb::Target;

/// Options of the `flash` sub command.
#[derive(Clone, Debug)]
pub struct FlashOptions {
    /// Image to program.
    pub elf: PathBuf,
    /// Crystal on `OSC_IN`; only affects the clock tree the firmware will see.
    pub hse_hz: u32,
}

impl Default for FlashOptions {
    fn default() -> Self {
        FlashOptions {
            elf: PathBuf::new(),
            hse_hz: 8_000_000,
        }
    }
}

/// Erases, programs and verifies the image, then reports what landed in flash.
pub fn run(options: FlashOptions) -> Result<(), String> {
    let image = ElfImage::open(&options.elf).map_err(|error| error.to_string())?;
    let mut target = Target::new(options.hse_hz, false)?;
    let report = target.program_image(&image)?;

    eprintln!(
        "flashed {} ({} bytes in {} pages, read back and verified)",
        options.elf.display(),
        report.bytes,
        report.pages
    );

    let Some((stack, reset)) = target.reset_vector() else {
        return Err("the image left no valid vector table in flash".to_string());
    };
    let entry = image
        .function_at(reset)
        .map(|symbol| symbol.name.clone())
        .unwrap_or_else(|| "<unknown>".to_string());
    eprintln!("vector table: sp=0x{stack:08X} reset=0x{reset:08X} ({entry})");
    eprintln!("start it with `gsimulator run {}`", options.elf.display());
    Ok(())
}
