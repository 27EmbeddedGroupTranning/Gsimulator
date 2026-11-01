//! Debug service: the front end's own virtual ST-Link.
//!
//! The GUI owns the simulated chip on the UI thread, so the server cannot live
//! in a thread of its own: the listening socket is polled once per frame and
//! every request is served by the very [`Simulation`] the canvas draws. That is
//! what makes a breakpoint freeze the picture on screen instead of some other
//! machine.
//!
//! The protocol itself (framing, packets, the device operations) lives in
//! `gsim-gdb`; this module only supplies the device ([`CoreTarget`]) and the
//! non-blocking loop around it.

use std::io;
use std::net::{Ipv4Addr, SocketAddr, TcpListener};

use eframe::egui;
use egui::{Color32, RichText};

use gsim_core::{AccessWidth, MemoryBus};
use gsim_cpu::StopReason;
use gsim_gdb::description;
use gsim_gdb::{
    Connection, DebugTarget, GdbServer, PointKind, Reply, ServerOptions, Stop, Waiting,
};
use gsim_stm32f1::map;
use gsim_stm32f1::Memory;

use crate::sim::{Simulation, Source};

/// Default port; it is the one OpenOCD and `gsimulator gdb` use as well.
pub const DEFAULT_PORT: u16 = 3333;
/// Requests served per frame, so a flood of packets cannot stall the UI.
const PACKETS_PER_FRAME: usize = 32;

const COLOR_OK: Color32 = Color32::from_rgb(120, 220, 140);
const COLOR_WARN: Color32 = Color32::from_rgb(220, 180, 90);
const COLOR_ERROR: Color32 = Color32::from_rgb(230, 110, 100);
const COLOR_DIM: Color32 = Color32::from_rgb(140, 148, 160);

const HELP: &str = "\
reset             reset the device and halt the core\n\
halt              stop the core\n\
regs              dump the core registers\n\
info              instruction, clock and interrupt counters\n\
peripherals       clock and state of every modelled block\n\
flash             flash geometry\n\
";

/// Why the core stopped, in the terms the remote protocol speaks.
fn describe(reason: StopReason, breakpoints: &[(u32, bool)]) -> Option<Stop> {
    Some(match reason {
        StopReason::InstructionLimit => return None,
        StopReason::Breakpoint(address) => Stop::Breakpoint {
            address,
            hardware: breakpoints
                .iter()
                .find(|(candidate, _)| *candidate == address)
                .is_some_and(|(_, hardware)| *hardware),
        },
        StopReason::Break => Stop::Break,
        StopReason::Halted => Stop::Halted,
        StopReason::Reset => Stop::Reset,
        StopReason::Debug => Stop::Halt,
    })
}

/// Stop reply for a target that failed while it was resumed.
fn fault_reply() -> String {
    format!("T{:02x}thread:1;", gsim_gdb::Signal::Segv.number())
}

/// The simulation as the debugger sees it: one core, its memory and the
/// breakpoints GDB asked for.
struct CoreTarget<'a> {
    sim: &'a mut Simulation,
    breakpoints: &'a mut Vec<(u32, bool)>,
}

impl CoreTarget<'_> {
    /// Bytes that can be touched before an access would cross a region border.
    fn room(&self, address: u32) -> usize {
        if Memory::is_flash(address) {
            (map::FLASH_BASE + map::FLASH_SIZE - address) as usize
        } else if Memory::is_sram(address) {
            (map::SRAM_BASE + map::SRAM_SIZE - address) as usize
        } else {
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
            self.sim
                .with_soc(|soc| soc.write(address, width, value))
                .map_err(|error| error.to_string())?;
            offset += bytes;
            address += bytes as u32;
        }
        Ok(())
    }
}

impl DebugTarget for CoreTarget<'_> {
    fn attach(&mut self) {
        // The GUI already reset from the vector table when it loaded the ELF;
        // attaching only has to stop the core.
        self.sim.debug_halt();
    }

    fn pc(&mut self) -> u32 {
        self.sim.debug_pc().unwrap_or(0)
    }

    fn halt(&mut self) {
        self.sim.debug_halt();
    }

    fn resume(&mut self) {
        self.sim.debug_resume();
    }

    fn step(&mut self) -> Result<Option<Stop>, String> {
        let reason = self.sim.debug_step();
        Ok(reason.and_then(|reason| describe(reason, self.breakpoints)))
    }

    fn hard_reset(&mut self) -> Result<(), String> {
        self.sim.debug_reset()
    }

    fn read_registers(&mut self) -> Vec<u32> {
        (0..description::REGISTER_COUNT)
            .map(|index| self.read_register(index))
            .collect()
    }

    fn read_register(&mut self, index: usize) -> u32 {
        let Some(cpu) = self.sim.debug_cpu() else {
            return 0;
        };
        if index == description::XPSR_REGISTER {
            return cpu.xpsr();
        }
        cpu.register(index as u16).unwrap_or(0)
    }

    fn write_registers(&mut self, values: &[u32]) {
        for (index, value) in values.iter().enumerate() {
            self.write_register(index, *value);
        }
    }

    fn write_register(&mut self, index: usize, value: u32) {
        let Some(cpu) = self.sim.debug_cpu() else {
            return;
        };
        if index == description::XPSR_REGISTER {
            cpu.set_xpsr(value);
        } else {
            cpu.set_register(index as u16, value);
        }
    }

    fn read_memory(&mut self, address: u32, length: u32) -> Result<Vec<u8>, String> {
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
                .sim
                .with_soc(|soc| soc.read(address, width))
                .map_err(|error| error.to_string())?;
            for index in 0..width.bytes() {
                data.push(((value >> (index * 8)) & 0xFF) as u8);
            }
            address += width.bytes();
        }
        Ok(data)
    }

    fn write_memory(&mut self, address: u32, data: &[u8]) -> Result<(), String> {
        // Flash goes through the same loader the ELF uses, everything else
        // through the bus, so a poke behaves like one on a real probe.
        let mut address = address;
        let mut offset = 0;
        while offset < data.len() {
            let length = (data.len() - offset).min(self.room(address));
            let chunk = &data[offset..offset + length];
            if Memory::is_flash(address) {
                self.sim
                    .with_soc(|soc| soc.load_image(address, chunk))
                    .map_err(|error| error.to_string())?;
            } else {
                self.write_words(address, chunk)?;
            }
            offset += length;
            address += length as u32;
        }
        Ok(())
    }

    fn flash_erase(&mut self, address: u32, _length: u32) -> Result<(), String> {
        if !Memory::is_flash(address) {
            return Err(format!("0x{address:08X} is not in the flash bank"));
        }
        // Programming goes through the loader, which overwrites whole bytes;
        // erasing only has to check that the range is real flash.
        Ok(())
    }

    fn flash_write(&mut self, address: u32, data: &[u8]) -> Result<(), String> {
        if !Memory::is_flash(address) {
            return Err(format!("0x{address:08X} is not in the flash bank"));
        }
        self.sim
            .with_soc(|soc| soc.load_image(address, data))
            .map_err(|error| error.to_string())
    }

    fn insert_breakpoint(&mut self, kind: PointKind, address: u32, _size: u32) -> bool {
        let address = address & !1;
        let hardware = kind == PointKind::Hardware;
        match self
            .breakpoints
            .iter_mut()
            .find(|(candidate, _)| *candidate == address)
        {
            Some(entry) => entry.1 = hardware,
            None => self.breakpoints.push((address, hardware)),
        }
        if let Some(cpu) = self.sim.debug_cpu() {
            cpu.add_breakpoint(address);
        }
        true
    }

    fn remove_breakpoint(&mut self, _kind: PointKind, address: u32, _size: u32) -> bool {
        let address = address & !1;
        self.breakpoints
            .retain(|(candidate, _)| *candidate != address);
        if let Some(cpu) = self.sim.debug_cpu() {
            cpu.remove_breakpoint(address);
        }
        true
    }

    fn insert_watchpoint(&mut self, _kind: PointKind, _address: u32, _size: u32) -> bool {
        // The core model has no address comparators, so the front end refuses
        // watchpoints instead of reporting hits it cannot observe.
        false
    }

    fn remove_watchpoint(&mut self, _address: u32, _size: u32) -> bool {
        true
    }

    fn monitor(&mut self, command: &str) -> String {
        match command.split_whitespace().next().unwrap_or("") {
            "" | "help" => HELP.to_string(),
            "reset" => match self.sim.debug_reset() {
                Ok(()) => "the device was reset and the core is halted\n".to_string(),
                Err(message) => format!("{message}\n"),
            },
            "halt" => {
                self.sim.debug_halt();
                "the core is halted\n".to_string()
            }
            "regs" | "registers" => self.register_dump(),
            "info" | "status" => self.summary(),
            "peripherals" => {
                let snapshot = self.sim.snapshot();
                let mut text = String::new();
                for block in &snapshot.peripherals {
                    text.push_str(&format!(
                        "{:<10} {:<10} {}\n",
                        block.name,
                        block.kind,
                        gsim_stm32f1::status::hz(block.clock_hz)
                    ));
                }
                text
            }
            "flash" => format!(
                "flash: {} KiB at 0x{:08X}, {} byte pages\n",
                map::FLASH_SIZE / 1024,
                map::FLASH_BASE,
                description::FLASH_PAGE_SIZE
            ),
            other => format!("unknown monitor command `{other}`; try `monitor help`\n"),
        }
    }
}

impl CoreTarget<'_> {
    fn register_dump(&mut self) -> String {
        const NAMES: [&str; 17] = [
            "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp",
            "lr", "pc", "xpsr",
        ];
        let registers = self.read_registers();
        let mut text = String::new();
        for (index, name) in NAMES.iter().enumerate() {
            text.push_str(&format!(
                "{name:<5}{:08X}\n",
                registers.get(index).copied().unwrap_or(0)
            ));
        }
        text
    }

    fn summary(&mut self) -> String {
        let (pc, instructions) = match self.sim.debug_cpu() {
            Some(cpu) => (cpu.pc(), cpu.instructions()),
            None => (0, 0),
        };
        let clocks = self.sim.clocks();
        format!(
            "instructions: {instructions}\n\
             simulated time: {:.3} ms\n\
             pc: 0x{pc:08X}\n\
             SYSCLK: {}\n",
            self.sim.now_ns() as f64 / 1_000_000.0,
            gsim_stm32f1::status::hz(clocks.sysclk_hz)
        )
    }
}

/// The listening socket plus the one client it serves, exactly like a probe.
pub struct DebugService {
    listener: Option<TcpListener>,
    connection: Option<Connection>,
    server: GdbServer,
    port: u16,
    peer: Option<SocketAddr>,
    /// `true` between GDB's `continue` and the stop reply.
    running: bool,
    /// Addresses armed by the debugger, with the kind GDB asked for.
    breakpoints: Vec<(u32, bool)>,
    error: Option<String>,
    /// Last packet exchanges, newest last, for the window's log.
    log: std::collections::VecDeque<String>,
}

/// Entries kept in the packet log.
const LOG_LINES: usize = 60;

impl Default for DebugService {
    fn default() -> Self {
        DebugService::new()
    }
}

impl DebugService {
    pub fn new() -> Self {
        DebugService {
            listener: None,
            connection: None,
            server: GdbServer::new(ServerOptions {
                bind: SocketAddr::from((Ipv4Addr::LOCALHOST, DEFAULT_PORT)),
                elf: None,
                hse_hz: 8_000_000,
                echo_uart: false,
                verbose: false,
            }),
            port: DEFAULT_PORT,
            peer: None,
            running: false,
            breakpoints: Vec::new(),
            error: None,
            log: std::collections::VecDeque::new(),
        }
    }

    /// Appends one line to the packet log shown in the window.
    fn record(&mut self, line: String) {
        if self.log.len() >= LOG_LINES {
            self.log.pop_front();
        }
        self.log.push_back(line);
    }

    /// Packet log, oldest first.
    pub fn log(&self) -> impl Iterator<Item = &str> {
        self.log.iter().map(String::as_str)
    }

    /// `true` while the debugger needs this frame loop to stay alive.
    pub fn needs_frames(&self) -> bool {
        self.listener.is_some() || self.connection.is_some()
    }

    pub fn is_listening(&self) -> bool {
        self.listener.is_some()
    }

    pub fn is_attached(&self) -> bool {
        self.connection.is_some()
    }

    /// `true` while the debugger's `continue` has not stopped yet.
    pub fn is_running(&self) -> bool {
        self.running
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn peer(&self) -> Option<SocketAddr> {
        self.peer
    }

    pub fn breakpoints(&self) -> usize {
        self.breakpoints.len()
    }

    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    pub fn set_error(&mut self, message: String) {
        self.error = Some(message);
    }

    /// Binds `127.0.0.1:port` and waits for a debugger.
    pub fn start(&mut self, port: u16) -> Result<(), String> {
        let address = SocketAddr::from((Ipv4Addr::LOCALHOST, port));
        let listener =
            TcpListener::bind(address).map_err(|error| format!("无法监听 {address}：{error}"))?;
        listener
            .set_nonblocking(true)
            .map_err(|error| format!("无法设置非阻塞监听：{error}"))?;
        // `start(0)` lets the OS pick a free port, which is what the tests use.
        let bound = listener
            .local_addr()
            .map_err(|error| format!("无法读取监听地址：{error}"))?;
        self.listener = Some(listener);
        self.port = bound.port();
        self.error = None;
        Ok(())
    }

    /// Stops listening and hands the core back to the run/pause button.
    pub fn stop(&mut self, sim: &mut Simulation) {
        self.release(sim);
        self.listener = None;
        self.peer = None;
    }

    /// Serves the debugger for one frame.
    ///
    /// Returns `true` when the debugger changed the run state, so the front end
    /// can refresh its widgets.
    pub fn poll(&mut self, sim: &mut Simulation) -> bool {
        let mut changed = self.accept_clients(sim);

        let mut keep = true;
        if let Some(mut link) = self.connection.take() {
            for _ in 0..PACKETS_PER_FRAME {
                match link.poll_wait() {
                    Ok(Some(Waiting::Packet(payload))) => {
                        changed = true;
                        if !self.serve(&mut link, &payload, sim) {
                            keep = false;
                            break;
                        }
                    }
                    Ok(Some(Waiting::Interrupt)) => {
                        changed = true;
                        self.interrupt(&mut link, sim);
                    }
                    Ok(Some(Waiting::Closed)) | Err(_) => {
                        keep = false;
                        break;
                    }
                    Ok(None) => break,
                }
            }
            if keep {
                self.connection = Some(link);
            } else {
                self.release(sim);
                self.peer = None;
                changed = true;
            }
        }

        // A breakpoint the running firmware hit is reported once.
        if self.running {
            if let Some(reason) = sim.take_debug_stop() {
                self.running = false;
                sim.set_debug_hold(true);
                self.report(reason);
                changed = true;
            }
        }
        changed
    }

    /// Takes every waiting client, replacing the one currently attached.
    ///
    /// VS Code (Cortex-Debug) opens a fresh connection for each session and the
    /// previous socket is often still open. Keeping it would leave the new
    /// debugger unanswered - GDB then reports "Ignoring packet error" after its
    /// first timeout - so the newest client always wins.
    fn accept_clients(&mut self, sim: &mut Simulation) -> bool {
        let mut changed = false;
        loop {
            let Some(listener) = self.listener.as_ref() else {
                return changed;
            };
            match listener.accept() {
                Ok((stream, peer)) => {
                    changed = true;
                    match Connection::new(stream) {
                        Ok(mut connection) => {
                            // The UI thread serves this socket, so it must never
                            // block: a stalled debugger fails the write instead
                            // of freezing the window.
                            if let Err(error) = connection.set_non_blocking(true) {
                                self.error = Some(format!("无法设置非阻塞连接：{error}"));
                                continue;
                            }
                            if self.connection.is_some() {
                                self.record(format!("{peer} 取代了上一个调试器连接"));
                                self.release(sim);
                            } else {
                                self.record(format!("{peer} 已连接"));
                            }
                            // A new session always starts with acknowledgements
                            // on: `QStartNoAckMode` from the previous debugger
                            // must not leak into this connection.
                            self.server.begin_session();
                            self.connection = Some(connection);
                            self.peer = Some(peer);
                            self.running = false;
                            self.error = None;
                            let mut target = CoreTarget {
                                sim,
                                breakpoints: &mut self.breakpoints,
                            };
                            target.attach();
                        }
                        Err(error) => {
                            self.error = Some(format!("连接 {peer} 失败：{error}"));
                        }
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => return changed,
                Err(error) => {
                    self.error = Some(format!("监听失败：{error}"));
                    self.listener = None;
                    return changed;
                }
            }
        }
    }

    /// Serves one packet; `false` closes the connection.
    fn serve(&mut self, link: &mut Connection, payload: &[u8], sim: &mut Simulation) -> bool {
        let request = packet_summary(payload);
        let reply = {
            let mut target = CoreTarget {
                sim,
                breakpoints: &mut self.breakpoints,
            };
            self.server.dispatch(&mut target, payload)
        };
        link.set_acknowledge(self.server.acknowledge());
        self.record(format!("← {request}  → {}", reply_label(&reply)));
        if std::env::var_os("GSIM_DEBUG_PACKETS").is_some() {
            eprintln!("[gui-st-link] <- {request} -> {}", reply_label(&reply));
        }

        match reply {
            Reply::None => true,
            Reply::Text(text) => link.send(&text).is_ok(),
            Reply::Lines(lines) => {
                for line in lines {
                    let line = format!("{line}\n");
                    if link
                        .send(&format!("O{}", gsim_gdb::rsp::to_hex(line.as_bytes())))
                        .is_err()
                    {
                        return false;
                    }
                }
                link.send("OK").is_ok()
            }
            Reply::Close => {
                let _ = link.send("OK");
                false
            }
            Reply::Resume => {
                // No acknowledgement here: like a real probe, the very next
                // packet on the wire is the stop reply. Sending `OK` first
                // makes some GDB versions report "Ignoring packet error".
                sim.debug_resume();
                if sim.debug_hold() {
                    // Nothing to run (no firmware loaded): answer straight away,
                    // a debugger that waits for a reply would hang forever.
                    self.running = false;
                    return link
                        .send(&self.server.stop_reply(Some(&Stop::Halt)))
                        .is_ok();
                }
                self.running = true;
                true
            }
            Reply::Step => {
                let stop = {
                    let mut target = CoreTarget {
                        sim,
                        breakpoints: &mut self.breakpoints,
                    };
                    target.step()
                };
                self.running = false;
                sim.set_debug_hold(true);
                let text = match stop {
                    Ok(stop) => self.server.stop_reply(stop.as_ref()),
                    Err(message) => {
                        self.error = Some(message);
                        fault_reply()
                    }
                };
                link.send(&text).is_ok()
            }
        }
    }

    /// Control-C: stop where the firmware is.
    fn interrupt(&mut self, link: &mut Connection, sim: &mut Simulation) {
        sim.debug_halt();
        self.running = false;
        self.record("← 0x03（Ctrl-C）→ T02".to_string());
        let _ = link.send(&self.server.stop_reply(Some(&Stop::Halt)));
    }

    /// Tells the attached debugger why the core stopped.
    fn report(&mut self, reason: StopReason) {
        let Some(stop) = describe(reason, &self.breakpoints) else {
            return;
        };
        let text = self.server.stop_reply(Some(&stop));
        if let Some(link) = self.connection.as_mut() {
            let _ = link.send(&text);
        }
    }

    /// Drops the client and every breakpoint it armed; the firmware runs on.
    fn release(&mut self, sim: &mut Simulation) {
        if let Some(mut link) = self.connection.take() {
            link.close();
        }
        self.running = false;
        for (address, _) in std::mem::take(&mut self.breakpoints) {
            if let Some(cpu) = sim.debug_cpu() {
                cpu.remove_breakpoint(address);
            }
        }
        sim.set_debug_hold(false);
    }
}

/// Printable, length-capped form of a request, for the window's log.
fn packet_summary(payload: &[u8]) -> String {
    const LIMIT: usize = 64;
    let mut text = String::new();
    for byte in payload.iter().take(LIMIT) {
        match byte {
            b'\n' => text.push_str("\\n"),
            0x20..=0x7E => text.push(*byte as char),
            _ => text.push_str(&format!("\\x{byte:02x}")),
        }
    }
    if payload.len() > LIMIT {
        text.push('…');
    }
    text
}

/// Short description of a reply, for the `GSIM_DEBUG_PACKETS` trace.
fn reply_label(reply: &Reply) -> String {
    match reply {
        Reply::None => "(unsupported)".to_string(),
        Reply::Text(text) => text.clone(),
        Reply::Lines(lines) => format!("{} console lines", lines.len()),
        Reply::Close => "(close)".to_string(),
        Reply::Resume => "(resume)".to_string(),
        Reply::Step => "(step)".to_string(),
    }
}

/// Window with the on/off switch, the port and the live state.
pub struct DebugWindow {
    /// `true` while the window is open.
    pub open: bool,
    service: DebugService,
    /// Port typed by the user; applied when the service is switched on.
    port_text: String,
}

impl Default for DebugWindow {
    fn default() -> Self {
        DebugWindow::new()
    }
}

impl DebugWindow {
    pub fn new() -> Self {
        DebugWindow {
            open: false,
            service: DebugService::new(),
            port_text: DEFAULT_PORT.to_string(),
        }
    }

    pub fn toggle(&mut self) {
        self.open = !self.open;
    }

    /// Live state of the service, for the toolbar.
    pub fn is_listening(&self) -> bool {
        self.service.is_listening()
    }

    /// Port the service listens on (or last listened on).
    pub fn port(&self) -> u16 {
        self.service.port()
    }

    /// Starts the service unless it is already listening.
    pub fn start_default(&mut self) {
        if self.service.is_listening() {
            return;
        }
        if let Err(error) = self.start(DEFAULT_PORT) {
            self.service.set_error(format!("{error}（端口被占用？）"));
        }
    }

    /// `true` while the debugger needs the frame loop to keep running.
    ///
    /// The socket is only serviced from `App::draw`, so the window must keep
    /// repainting even while the canvas is paused - otherwise GDB waits for a
    /// reply it will never get and reports a packet error.
    pub fn needs_frames(&self) -> bool {
        self.service.needs_frames()
    }

    /// Serves the debugger; called once per frame, whether or not the window is
    /// open, so a running session is never interrupted by closing it.
    pub fn poll(&mut self, sim: &mut Simulation) -> bool {
        self.service.poll(sim)
    }

    /// Binds `port` (`0` picks a free one) and waits for a debugger.
    pub fn start(&mut self, port: u16) -> Result<(), String> {
        match self.service.start(port) {
            Ok(()) => {
                self.port_text = self.service.port().to_string();
                Ok(())
            }
            Err(error) => {
                self.service.set_error(error.clone());
                Err(error)
            }
        }
    }

    /// Stops listening and hands the core back to the run/pause button.
    pub fn stop(&mut self, sim: &mut Simulation) {
        self.service.stop(sim);
    }

    /// Draws the window; `sim` is only used by the on/off switch.
    pub fn show(&mut self, ctx: &egui::Context, sim: &mut Simulation) {
        if !self.open {
            return;
        }
        let mut open = self.open;
        egui::Window::new("🐞 调试服务")
            .id(egui::Id::new("debug-service"))
            .open(&mut open)
            .default_size([420.0, 300.0])
            .default_pos([300.0, 140.0])
            .resizable(true)
            .show(ctx, |ui| self.body(ui, sim));
        self.open = open;
    }

    fn body(&mut self, ui: &mut egui::Ui, sim: &mut Simulation) {
        let listening = self.service.is_listening();
        let mut enabled = listening;
        if ui
            .checkbox(&mut enabled, "开启调试服务（虚拟 ST-Link）")
            .on_hover_text("调试器和画布共用同一个仿真：断点命中时画面就停在那里")
            .changed()
        {
            if enabled {
                match self.port_text.trim().parse::<u16>() {
                    Ok(port) if port != 0 => {
                        let _ = self.start(port);
                    }
                    _ => self
                        .service
                        .set_error("端口必须是 1–65535 之间的数字".to_string()),
                }
            } else {
                self.stop(sim);
            }
        }

        ui.horizontal(|ui| {
            ui.label("端口");
            ui.add(
                egui::TextEdit::singleline(&mut self.port_text)
                    .desired_width(70.0)
                    .hint_text("3333"),
            );
            ui.label(
                RichText::new("127.0.0.1")
                    .monospace()
                    .small()
                    .color(COLOR_DIM),
            );
            if self.service.is_listening()
                && self.port_text.trim() != self.service.port().to_string()
            {
                ui.label(
                    RichText::new(format!("当前 {}", self.service.port()))
                        .small()
                        .color(COLOR_WARN),
                );
            }
        });

        ui.separator();
        if let Some(error) = self.service.error() {
            ui.label(RichText::new(error.to_string()).color(COLOR_ERROR));
        }
        if !self.service.is_listening() {
            ui.label(RichText::new("未开启").color(COLOR_DIM));
        } else if let Some(peer) = self.service.peer() {
            ui.label(
                RichText::new(format!("已连接 {peer}"))
                    .color(COLOR_OK)
                    .strong(),
            );
            ui.label(
                RichText::new(if self.service.is_running() {
                    "核心：运行中"
                } else {
                    "核心：已暂停（调试器持有）"
                })
                .color(if self.service.is_running() {
                    COLOR_OK
                } else {
                    COLOR_WARN
                }),
            );
            ui.label(
                RichText::new(format!(
                    "断点 {} · 监视点不支持",
                    self.service.breakpoints()
                ))
                .small()
                .color(COLOR_DIM),
            );
        } else {
            ui.label(
                RichText::new(format!("监听 127.0.0.1:{}", self.service.port()))
                    .color(COLOR_OK)
                    .strong(),
            );
            ui.label(RichText::new("等待调试器连接…").small().color(COLOR_DIM));
        }

        ui.separator();
        if sim.debug_pc().is_none() {
            ui.label(
                RichText::new("⚠ 还没有加载固件：断点永远不会命中")
                    .color(COLOR_ERROR)
                    .strong(),
            );
            ui.label(
                RichText::new("先用工具栏「📂 加载 ELF…」装载固件，再连调试器。")
                    .small()
                    .color(COLOR_DIM),
            );
        } else {
            let firmware = match sim.source() {
                Source::Firmware(path) => Some(path.clone()),
                _ => None,
            };
            let name = firmware
                .as_ref()
                .and_then(|path| path.file_name())
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_else(|| "固件".to_string());
            let pc = sim.debug_pc().unwrap_or_default();
            ui.label(
                RichText::new(format!("{name}  ·  PC = 0x{pc:08X}"))
                    .monospace()
                    .small()
                    .color(COLOR_OK),
            )
            .on_hover_text(
                firmware
                    .as_ref()
                    .map(|path| {
                        format!(
                            "{}\n必须和 launch.json 里的 executable 是同一个文件，否则断点地址对不上。",
                            path.display()
                        )
                    })
                    .unwrap_or_default(),
            );
            ui.label(
                RichText::new(
                    "断点、单步、变量都由这里的仿真执行；附加后先 monitor reset halt 复位再下断点。",
                )
                .small()
                .color(COLOR_DIM),
            );
        }

        ui.separator();
        ui.label(RichText::new("连接方式（Cortex-Debug 用「附加」配置）").strong());
        ui.label(
            RichText::new(format!(
                "target extended-remote 127.0.0.1:{}\nmonitor reset halt\nbreak main",
                self.service.port()
            ))
            .monospace()
            .small(),
        );

        let entries: Vec<String> = self.service.log().map(str::to_string).collect();
        egui::CollapsingHeader::new(format!("数据包日志（最近 {} 条）", entries.len()))
            .id_salt("gsim-debug-log")
            .default_open(false)
            .show(ui, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("gsim-debug-log-scroll")
                    .max_height(160.0)
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        if entries.is_empty() {
                            ui.label(RichText::new("（还没有收发）").small().color(COLOR_DIM));
                        }
                        for entry in entries.iter().rev() {
                            ui.label(RichText::new(entry).monospace().small());
                        }
                    });
            });
    }
}
