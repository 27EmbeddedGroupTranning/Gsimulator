//! End-to-end check with the real `arm-none-eabi-gdb`.
//!
//! It flashes, breaks, runs and steps the same firmware the `gsim-cpu` tests use,
//! so the whole path (GDB → remote protocol → device model) is exercised. On a
//! machine without the cross toolchain the test is skipped instead of failing.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::Command;

use gsim_gdb::{GdbServer, ServerOptions};

const GCC: &str = "arm-none-eabi-gcc";
const GDB: &str = "arm-none-eabi-gdb";

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
    let elf = Path::new(env!("CARGO_TARGET_TMPDIR")).join("gdb-firmware.elf");
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
fn real_gdb_flashes_breaks_and_runs_the_simulator() {
    let Some(elf) = firmware() else {
        eprintln!("skipping: {GCC} is not installed");
        return;
    };
    if !installed(GDB) {
        eprintln!("skipping: {GDB} is not installed");
        return;
    }

    let (listener, server) = GdbServer::bind(ServerOptions {
        bind: SocketAddr::from(([127, 0, 0, 1], 0)),
        echo_uart: false,
        ..ServerOptions::default()
    })
    .expect("the loopback socket binds");
    let address = listener.local_addr().expect("a bound address");
    std::thread::spawn(move || {
        let _ = server.serve(listener);
    });

    let image = elf.display().to_string().replace('\\', "/");
    let output = Command::new(GDB)
        .args(["--batch", "-nx", "-ex", "set confirm off"])
        .arg("-ex")
        .arg(format!("file {image}"))
        .arg("-ex")
        .arg(format!("target extended-remote {address}"))
        .args(["-ex", "load"])
        .args(["-ex", "break main"])
        .args(["-ex", "run"])
        .args(["-ex", "info registers pc"])
        .args(["-ex", "stepi"])
        .args(["-ex", "info registers pc"])
        .args(["-ex", "monitor info"])
        .args(["-ex", "detach"])
        // Without this GDB reads the harness' stdin as a command file.
        .stdin(std::process::Stdio::null())
        .output()
        .expect("gdb starts");

    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        text.contains("load size"),
        "gdb did not report a download:\n{text}"
    );
    assert!(
        text.contains("Breakpoint 1, main"),
        "gdb did not stop in main:\n{text}"
    );
    assert!(
        text.contains("instructions:"),
        "monitor info produced no output:\n{text}"
    );
    assert!(
        text.contains("<main"),
        "the pc should be inside main after the breakpoint:\n{text}"
    );
}
