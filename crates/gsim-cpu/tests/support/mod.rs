//! Shared helpers for the firmware execution tests.

#![allow(dead_code)]

use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::rc::Rc;

use gsim_core::Event;
use gsim_stm32f1::Soc;

/// Cross compiler used for the test firmwares.
const TOOLCHAIN: &str = "arm-none-eabi-gcc";

/// Collects `UartTx` events (and everything else) emitted by the SoC.
#[derive(Clone, Default)]
pub struct EventLog(Rc<RefCell<Vec<Event>>>);

impl EventLog {
    pub fn new() -> Self {
        EventLog(Rc::new(RefCell::new(Vec::new())))
    }

    pub fn attach(&self, soc: &mut Soc) {
        let log = Rc::clone(&self.0);
        soc.add_event_sink(Box::new(move |event: &Event| {
            log.borrow_mut().push(event.clone())
        }));
    }

    pub fn events(&self) -> Vec<Event> {
        self.0.borrow().clone()
    }

    pub fn uart(&self, unit: u8) -> Vec<u8> {
        self.0
            .borrow()
            .iter()
            .filter_map(|event| match event {
                Event::UartTx { unit: u, byte } if *u == unit => Some(*byte),
                _ => None,
            })
            .collect()
    }

    pub fn uart_text(&self, unit: u8) -> String {
        String::from_utf8_lossy(&self.uart(unit)).to_string()
    }

    pub fn spi(&self, unit: u8) -> Vec<(u16, u16)> {
        self.0
            .borrow()
            .iter()
            .filter_map(|event| match event {
                Event::SpiTransfer { unit: u, tx, rx } if *u == unit => Some((*tx, *rx)),
                _ => None,
            })
            .collect()
    }
}

/// True when the ARM cross compiler is available on this machine.
pub fn toolchain_available() -> bool {
    Command::new(TOOLCHAIN)
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

/// Compiles `tests/firmware/<name>.c` into `target/tmp/<name>.elf`.
///
/// Returns `None` when the cross compiler is not installed; the caller then
/// skips the test instead of failing on a machine without the toolchain.
pub fn build_firmware(name: &str) -> Option<PathBuf> {
    if !toolchain_available() {
        return None;
    }
    let source_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/firmware");
    let output = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{name}.elf"));
    let status = Command::new(TOOLCHAIN)
        .current_dir(&source_dir)
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
        .arg(&output)
        .args(["startup.c", &format!("{name}.c")])
        .status()
        .expect("cannot run the ARM cross compiler");
    assert!(status.success(), "firmware {name} failed to build");
    Some(output)
}
