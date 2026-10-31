//! Checks the `flash` path: erase, program and verify a real image.
//!
//! The firmware comes from `gsim-cpu`'s test sources, so the test is skipped on
//! machines without the cross compiler instead of failing.

use std::path::{Path, PathBuf};
use std::process::Command;

use gsim_cpu::ElfImage;
use gsim_gdb::Target;

const GCC: &str = "arm-none-eabi-gcc";

fn installed(tool: &str) -> bool {
    Command::new(tool)
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

/// Builds the shared smoke firmware; `None` without a cross compiler.
fn firmware() -> Option<PathBuf> {
    if !installed(GCC) {
        return None;
    }
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("../gsim-cpu/tests/firmware");
    let elf = Path::new(env!("CARGO_TARGET_TMPDIR")).join("flash-firmware.elf");
    let status = Command::new(GCC)
        .current_dir(&source)
        .args([
            "-mcpu=cortex-m3",
            "-mthumb",
            "-O1",
            "-g",
            "-ffreestanding",
            "-fno-builtin",
            "-nostdlib",
            "-nostartfiles",
            "-T",
            "link.ld",
            "-Wl,--gc-sections",
            "-o",
        ])
        .arg(&elf)
        .args(["startup.c", "main.c"])
        .status()
        .expect("the cross compiler starts");
    status.success().then_some(elf)
}

#[test]
fn a_real_image_is_erased_programmed_and_verified() {
    let Some(path) = firmware() else {
        eprintln!("skipping: {GCC} is not installed");
        return;
    };
    let image = ElfImage::open(&path).expect("the built firmware is a valid ELF");
    let mut target = Target::new(8_000_000, false).expect("the chip model starts");

    let report = target
        .program_image(&image)
        .expect("the image is programmed");

    let flash_bytes: u32 = image
        .segments()
        .iter()
        .filter(|segment| segment.is_flash())
        .map(|segment| segment.size())
        .sum();
    assert_eq!(report.bytes, flash_bytes, "every flash byte is programmed");
    assert!(report.pages >= 1, "at least one page is erased");

    // Read the whole flash image back: this is what `verify` means.
    for segment in image.segments().iter().filter(|segment| segment.is_flash()) {
        let read = target
            .read_memory(segment.load_address, segment.size())
            .expect("flash is readable");
        assert_eq!(
            read, segment.bytes,
            "segment at 0x{:08X} matches the ELF",
            segment.load_address
        );
    }

    // The vector table is in place, so the core can be reset from it.
    let (stack, reset) = target.reset_vector().expect("a valid vector table");
    assert_eq!(reset, image.entry() & !1);
    assert!(
        (0x2000_0000..=0x2000_0000 + 20 * 1024).contains(&stack),
        "sp=0x{stack:08X} points at the top of SRAM"
    );
    assert_eq!(
        image.function_at(reset).map(|symbol| symbol.name.as_str()),
        Some("Reset_Handler"),
        "the reset vector names the reset handler"
    );

    target
        .reset()
        .expect("the core restarts from the vector table");
    assert_eq!(target.pc(), reset);
}
