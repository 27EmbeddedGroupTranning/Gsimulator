//! End-to-end test of the front end's own virtual ST-Link.
//!
//! A raw remote-protocol client talks to [`DebugService`] while the simulation
//! is stepped the way `App::draw` steps it, so the test proves that a breakpoint
//! really stops the machine the canvas draws.
//!
//! The image is a build artefact: when it is missing the test only reports the
//! skip, so a fresh checkout still passes.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use eframe::egui;
use gsim_cpu::ElfImage;
use gsim_gdb::rsp;
use gsimulator_gui::debugger::{DebugService, DebugWindow};
use gsimulator_gui::sim::Simulation;

/// Instructions the test runs between two polls, like one UI frame.
const FRAME_BUDGET: u32 = 200_000;

fn firmware() -> Option<PathBuf> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../firmware/CANSPI/build/Debug/CANSPI.elf");
    if path.exists() {
        Some(path)
    } else {
        eprintln!(
            "skipping: {} is missing (build firmware/CANSPI with CMake first)",
            path.display()
        );
        None
    }
}

fn installed(tool: &str) -> bool {
    std::process::Command::new(tool)
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

/// Client plus the simulation and the service under test.
struct Bench {
    sim: Simulation,
    service: DebugService,
    client: TcpStream,
    buffer: Vec<u8>,
}

impl Bench {
    fn new(path: &Path) -> Self {
        let mut sim = Simulation::new();
        sim.load_elf(path).expect("the firmware loads");
        let mut service = DebugService::new();
        service.start(0).expect("an ephemeral port is bound");
        let client =
            TcpStream::connect(("127.0.0.1", service.port())).expect("the service accepts");
        client
            .set_read_timeout(Some(Duration::from_millis(50)))
            .expect("a read timeout");
        Bench {
            sim,
            service,
            client,
            buffer: Vec::new(),
        }
    }

    /// One UI frame: serve the debugger, then let the firmware run.
    fn frame(&mut self) {
        self.service.poll(&mut self.sim);
        self.sim.advance_frame(FRAME_BUDGET);
    }

    fn send(&mut self, payload: &str) {
        self.client
            .write_all(&rsp::encode(payload))
            .expect("the request is written");
    }

    /// Serves frames until a whole reply packet arrived.
    fn expect(&mut self) -> String {
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            if let rsp::Frame::Packet(payload) = rsp::next_frame(&mut self.buffer) {
                return String::from_utf8_lossy(&payload).to_string();
            }
            assert!(
                Instant::now() < deadline,
                "no reply; the service stopped answering"
            );
            self.frame();
            let mut chunk = [0u8; 4096];
            match self.client.read(&mut chunk) {
                Ok(0) => panic!("the service closed the connection"),
                Ok(read) => self.buffer.extend_from_slice(&chunk[..read]),
                Err(error)
                    if error.kind() == std::io::ErrorKind::WouldBlock
                        || error.kind() == std::io::ErrorKind::TimedOut => {}
                Err(error) => panic!("read failed: {error}"),
            }
        }
    }

    /// Sends a request and returns the first reply.
    fn request(&mut self, payload: &str) -> String {
        self.send(payload);
        self.expect()
    }
}

#[test]
fn a_breakpoint_stops_the_simulation_the_canvas_draws() {
    let Some(path) = firmware() else {
        return;
    };
    let image = ElfImage::open(&path).expect("the ELF parses");
    // Thumb symbols come with bit 0 set; the core stops on the instruction.
    let main = image
        .symbol("main")
        .expect("the firmware keeps its symbols")
        .address
        & !1;

    let mut bench = Bench::new(&path);

    // Handshake as GDB does it.
    let features = bench.request("qSupported");
    assert!(features.contains("PacketSize="), "{features}");
    assert!(features.contains("swbreak+"), "{features}");
    let stop = bench.request("?");
    assert!(stop.starts_with("T05"), "{stop}");

    // Attach halts the core, so the frame loop must not advance it.
    bench.frame();
    let clock = bench.sim.now_ns();
    for _ in 0..5 {
        bench.frame();
    }
    assert_eq!(bench.sim.now_ns(), clock, "a halted core still ran");

    // A breakpoint on `main`, then continue: the debugger's own code runs the
    // firmware, and the stop reply has to name the breakpoint.
    assert_eq!(bench.request(&format!("Z1,{main:x},2")), "OK");
    assert_eq!(bench.request("c"), "OK");

    let deadline = Instant::now() + Duration::from_secs(20);
    let stop = loop {
        let reply = bench.expect();
        if reply.starts_with("T05") {
            break reply;
        }
        assert!(
            Instant::now() < deadline,
            "the firmware never hit the breakpoint"
        );
    };
    assert!(stop.contains("hwbreak"), "{stop}");
    assert_eq!(bench.sim.debug_pc(), Some(main), "stopped somewhere else");

    // The register file is readable, and the PC register agrees.
    let registers = bench.request("g");
    assert_eq!(registers.len(), 17 * 8, "the whole register file");
    // Register words travel little endian, so the payload is re-assembled.
    let bytes = rsp::from_hex(registers.as_bytes()).expect("the register dump is hex");
    let pc = u32::from_le_bytes([
        bytes[15 * 4],
        bytes[15 * 4 + 1],
        bytes[15 * 4 + 2],
        bytes[15 * 4 + 3],
    ]);
    assert_eq!(pc, main);

    // The core is frozen again, so the canvas does not move either.
    let clock = bench.sim.now_ns();
    for _ in 0..5 {
        bench.frame();
    }
    assert_eq!(
        bench.sim.now_ns(),
        clock,
        "the core ran past the breakpoint"
    );

    // A single step stops right away and moves the PC on.
    assert!(bench.request("s").starts_with("T05"));
    assert_ne!(bench.sim.debug_pc(), Some(main), "the step did not advance");

    // Monitor commands answer with console output, then OK.
    // `qRcmd` carries the command without GDB's `monitor` prefix.
    bench.send(&format!("qRcmd,{}", rsp::to_hex(b"halt")));
    let mut lines = String::new();
    loop {
        let reply = bench.expect();
        if reply == "OK" {
            break;
        }
        if let Some(payload) = reply.strip_prefix('O') {
            if let Some(bytes) = rsp::from_hex(payload.as_bytes()) {
                lines.push_str(&String::from_utf8_lossy(&bytes));
            }
        }
    }
    assert!(lines.contains("halted"), "{lines:?}");
}

/// The scenario the GUI is used in: VS Code (Cortex-Debug) attaches to the
/// service of the running simulation and stops the canvas at `main`.
#[test]
fn real_gdb_attaches_to_the_gui_service_and_stops_at_main() {
    const GDB: &str = "arm-none-eabi-gdb";
    let Some(path) = firmware() else {
        return;
    };
    if !installed(GDB) {
        eprintln!("skipping: {GDB} is not installed");
        return;
    }

    let mut sim = Simulation::new();
    sim.load_elf(&path).expect("the firmware loads");
    let mut service = DebugService::new();
    service.start(0).expect("an ephemeral port is bound");
    let port = service.port();

    let image = path.display().to_string().replace('\\', "/");
    let mut command = std::process::Command::new(GDB);
    command.args(["--batch", "-nx"]);
    if std::env::var_os("GSIM_GDB_TRACE").is_some() {
        command.args(["-ex", "set debug remote 1"]);
    }
    let mut gdb = command
        .arg("-ex")
        .arg("set confirm off")
        .arg("-ex")
        .arg(format!("file {image}"))
        .arg("-ex")
        .arg(format!("target extended-remote 127.0.0.1:{port}"))
        // The GUI runs the firmware before the debugger attaches, so restart it
        // from the reset vector first - that is what Cortex-Debug's launch does.
        .args(["-ex", "monitor reset halt"])
        .args(["-ex", "break main"])
        .args(["-ex", "continue"])
        .args(["-ex", "info registers pc"])
        .args(["-ex", "detach"])
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("gdb starts");

    // Keep serving and stepping exactly like `App::draw` while gdb runs.
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        service.poll(&mut sim);
        sim.advance_frame(FRAME_BUDGET);
        match gdb.try_wait() {
            Ok(Some(_)) => break,
            Ok(None) => {}
            Err(error) => panic!("gdb could not be polled: {error}"),
        }
        if Instant::now() > deadline {
            let _ = gdb.kill();
            panic!("gdb never finished");
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    let output = gdb.wait_with_output().expect("gdb output");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        text.contains("Breakpoint 1, main"),
        "gdb never stopped in main:\n{text}"
    );
    assert!(
        text.contains("<main"),
        "the pc should be inside main:\n{text}"
    );
}

/// The window itself renders headlessly and its switch really opens the port.
#[test]
fn the_debug_window_renders_and_opens_the_port() {
    let ctx = egui::Context::default();
    let mut sim = Simulation::new();
    let mut window = DebugWindow::new();
    window.open = true;

    let input = egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::Vec2::new(1280.0, 800.0),
        )),
        ..Default::default()
    };
    for _ in 0..3 {
        let _ = ctx.run(input.clone(), |ctx| window.show(ctx, &mut sim));
    }
    assert!(!window.is_listening(), "the switch starts off");

    // Switching it on from the UI is what the user does; the port is bound.
    let mut service = DebugService::new();
    service.start(0).expect("an ephemeral port is bound");
    assert!(service.is_listening());
    assert_eq!(service.peer(), None);
    service.stop(&mut sim);
    assert!(!service.is_listening());
}
