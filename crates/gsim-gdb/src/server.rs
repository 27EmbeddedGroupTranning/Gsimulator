//! The debug server: a GDB remote protocol endpoint that behaves like a probe
//! attached to the simulated chip.
//!
//! One thread owns the simulated device, so a resume runs the core in slices
//! between two polls of the socket. That keeps control-C responsive without
//! needing any shared state or extra threads.

use std::io;
use std::net::{SocketAddr, TcpListener};
use std::path::PathBuf;

use crate::connection::{Connection, Poll, Waiting};
use crate::description;
use crate::rsp;
use crate::target::{PointKind, Stop, Target};

/// Instructions executed between two polls of the connection.
const RESUME_SLICE: u64 = 100_000;

/// Largest packet payload the server accepts, advertised through `qSupported`.
const PACKET_SIZE: usize = 0x4000;

/// Thread list served through `qXfer:threads:read`; the core is one thread.
const THREADS_XML: &str = "<?xml version=\"1.0\"?>\
<threads><thread id=\"1\" name=\"Cortex-M3\"/></threads>";

/// Options of a [`GdbServer`].
#[derive(Clone, Debug)]
pub struct ServerOptions {
    /// Address the server listens on; OpenOCD uses port 3333 as well.
    pub bind: SocketAddr,
    /// ELF image loaded before the first connection, if any.
    pub elf: Option<PathBuf>,
    /// Crystal on `OSC_IN`, used by the RCC model.
    pub hse_hz: u32,
    /// Copy the firmware's UART output to stdout.
    pub echo_uart: bool,
    /// Log protocol activity on stderr.
    pub verbose: bool,
}

impl Default for ServerOptions {
    fn default() -> Self {
        ServerOptions {
            bind: SocketAddr::from(([127, 0, 0, 1], 3333)),
            elf: None,
            hse_hz: 8_000_000,
            echo_uart: true,
            verbose: false,
        }
    }
}

/// What to send back for one request.
enum Reply {
    /// Nothing: the request was optional or unknown.
    None,
    /// A payload.
    Text(String),
    /// The session is over.
    Close,
}

/// The GDB server itself.
pub struct GdbServer {
    options: ServerOptions,
    memory_map: String,
}

impl GdbServer {
    /// Creates a server; call [`GdbServer::bind`] or [`GdbServer::serve`] next.
    pub fn new(options: ServerOptions) -> Self {
        GdbServer {
            options,
            memory_map: description::memory_map(),
        }
    }

    /// Binds the listening socket described by `options`.
    pub fn bind(options: ServerOptions) -> io::Result<(TcpListener, Self)> {
        let listener = TcpListener::bind(options.bind)?;
        Ok((listener, GdbServer::new(options)))
    }

    /// Serves GDB connections until the listener fails.
    ///
    /// The simulated chip survives disconnects, so a debugger can detach,
    /// reconnect and find the firmware exactly where it was.
    pub fn serve(mut self, listener: TcpListener) -> io::Result<()> {
        let address = listener.local_addr()?;
        eprintln!("virtual ST-Link listening on {address}");
        eprintln!(
            "  arm-none-eabi-gdb <firmware.elf> -ex 'target extended-remote {address}'"
        );

        let mut target: Option<Target> = None;
        for stream in listener.incoming() {
            let stream = stream?;
            let mut connection = Connection::new(stream)?;
            if target.is_none() {
                target = Some(self.build_target()?);
            }
            let Some(state) = target.as_mut() else {
                continue;
            };
            match self.session(&mut connection, state) {
                Ok(()) => self.note("client detached"),
                Err(error) => eprintln!("error: {error}"),
            }
        }
        Ok(())
    }

    fn build_target(&self) -> io::Result<Target> {
        let mut target =
            Target::new(self.options.hse_hz, self.options.echo_uart).map_err(io::Error::other)?;
        if let Some(path) = &self.options.elf {
            target.load_elf(path).map_err(|message| {
                io::Error::other(format!("{}: {message}", path.display()))
            })?;
            eprintln!("loaded {}", path.display());
        }
        Ok(target)
    }

    fn session(&mut self, connection: &mut Connection, target: &mut Target) -> io::Result<()> {
        target.attach();
        self.note(&format!(
            "attached: core halted at 0x{:08X}",
            target.pc()
        ));
        loop {
            match connection.wait()? {
                Waiting::Closed => return Ok(()),
                Waiting::Interrupt => continue,
                Waiting::Packet(payload) => {
                    if self.options.verbose {
                        eprintln!("[gsim-gdb] <- {}", preview(&payload));
                    }
                    let reply = self.dispatch(connection, target, &payload)?;
                    match reply {
                        Reply::None => {
                            if self.options.verbose {
                                eprintln!("[gsim-gdb] -> (unsupported)");
                            }
                        }
                        Reply::Text(text) => {
                            if self.options.verbose {
                                eprintln!("[gsim-gdb] -> {}", preview(text.as_bytes()));
                            }
                            connection.send(&text)?;
                        }
                        Reply::Close => {
                            connection.close();
                            return Ok(());
                        }
                    }
                }
            }
        }
    }

    fn dispatch(
        &mut self,
        connection: &mut Connection,
        target: &mut Target,
        payload: &[u8],
    ) -> io::Result<Reply> {
        // Packets with a binary payload are handled before the text paths.
        if let Some(rest) = payload.strip_prefix(b"X".as_slice()) {
            return Ok(self.binary_write(target, rest));
        }
        if let Some(rest) = payload.strip_prefix(b"vFlashWrite:".as_slice()) {
            return Ok(self.flash_write(target, rest));
        }

        let text = String::from_utf8_lossy(payload);
        let text = text.as_ref();
        let Some(head) = text.as_bytes().first().copied() else {
            return Ok(Reply::None);
        };

        match head {
            b'?' => Ok(Reply::Text(self.stop_reply(None))),
            b'g' if text.len() == 1 => Ok(Reply::Text(self.read_registers(target))),
            b'G' => Ok(Reply::Text(self.write_registers(target, &text[1..]))),
            b'p' => Ok(Reply::Text(self.read_register(target, &text[1..]))),
            b'P' => Ok(Reply::Text(self.write_register(target, &text[1..]))),
            b'm' => Ok(Reply::Text(self.read_memory(target, &text[1..]))),
            b'M' => Ok(Reply::Text(self.write_memory(target, &text[1..]))),
            b'c' | b'C' => self.continue_target(connection, target),
            b's' | b'S' => self.step_target(target),
            b'Z' => Ok(Reply::Text(self.insert_point(target, &text[1..]))),
            b'z' => Ok(Reply::Text(self.remove_point(target, &text[1..]))),
            b'k' => Ok(Reply::Close),
            b'D' => {
                target.halt();
                Ok(Reply::Text("OK".to_string()))
            }
            b'!' | b'H' | b'T' => Ok(Reply::Text("OK".to_string())),
            b'R' => {
                target.halt();
                match target.hard_reset() {
                    Ok(()) => Ok(Reply::Text("OK".to_string())),
                    Err(message) => {
                        self.note(&message);
                        Ok(Reply::Text("E01".to_string()))
                    }
                }
            }
            b'q' => self.query(connection, target, text),
            b'Q' => Ok(self.set_query(connection, text)),
            b'v' => self.extended(connection, target, text),
            _ => Ok(Reply::None),
        }
    }

    // ---------------------------------------------------------------- resume

    fn continue_target(
        &mut self,
        connection: &mut Connection,
        target: &mut Target,
    ) -> io::Result<Reply> {
        target.resume();
        loop {
            match target.run(RESUME_SLICE) {
                Ok(Some(stop)) => {
                    self.note(&format!("stopped: {stop:?}"));
                    return Ok(Reply::Text(self.stop_reply(Some(&stop))));
                }
                Ok(None) => match connection.poll()? {
                    Poll::Idle => {}
                    Poll::Interrupt => {
                        target.halt();
                        self.note("stopped: control-C");
                        return Ok(Reply::Text(self.stop_reply(Some(&Stop::Halt))));
                    }
                    Poll::Closed => return Ok(Reply::Close),
                },
                Err(message) => {
                    eprintln!("error: {message}");
                    target.halt();
                    return Ok(Reply::Text(format!(
                        "T{:02x}thread:1;",
                        crate::target::Signal::Segv.number()
                    )));
                }
            }
        }
    }

    fn step_target(&mut self, target: &mut Target) -> io::Result<Reply> {
        target.resume();
        match target.step() {
            Ok(stop) => Ok(Reply::Text(self.stop_reply(stop.as_ref()))),
            Err(message) => {
                eprintln!("error: {message}");
                target.halt();
                Ok(Reply::Text(format!(
                    "T{:02x}thread:1;",
                    crate::target::Signal::Segv.number()
                )))
            }
        }
    }

    fn stop_reply(&self, stop: Option<&Stop>) -> String {
        let signal = stop.map_or(5, |stop| stop.signal().number());
        let mut reply = format!("T{signal:02x}thread:1;");
        match stop {
            Some(Stop::Breakpoint { hardware: true, .. }) => reply.push_str("hwbreak:;"),
            Some(Stop::Breakpoint { hardware: false, .. }) => reply.push_str("swbreak:;"),
            Some(Stop::Watchpoint { address }) => {
                reply.push_str(&format!("awatch:{address:x};"));
            }
            _ => {}
        }
        reply
    }

    // ------------------------------------------------------------- registers

    fn read_registers(&self, target: &Target) -> String {
        target
            .read_registers()
            .iter()
            .map(|value| rsp::word_to_hex(*value))
            .collect()
    }

    fn write_registers(&mut self, target: &mut Target, payload: &str) -> String {
        let Some(values) = rsp::words_from_hex(payload.as_bytes()) else {
            return "E01".to_string();
        };
        if values.len() < description::REGISTER_COUNT {
            return "E01".to_string();
        }
        target.write_registers(&values);
        "OK".to_string()
    }

    fn read_register(&self, target: &Target, payload: &str) -> String {
        let Some(index) = rsp::parse_number(payload) else {
            return "E01".to_string();
        };
        if index as usize >= description::REGISTER_COUNT {
            return "E01".to_string();
        }
        rsp::word_to_hex(target.read_register(index as usize))
    }

    fn write_register(&mut self, target: &mut Target, payload: &str) -> String {
        let Some((index, value)) = payload.split_once('=') else {
            return "E01".to_string();
        };
        let Some(index) = rsp::parse_number(index) else {
            return "E01".to_string();
        };
        if index as usize >= description::REGISTER_COUNT {
            return "E01".to_string();
        }
        let Some(bytes) = rsp::from_hex(value.as_bytes()) else {
            return "E01".to_string();
        };
        let mut register = [0u8; 4];
        let length = bytes.len().min(4);
        register[..length].copy_from_slice(&bytes[..length]);
        target.write_register(index as usize, u32::from_le_bytes(register));
        "OK".to_string()
    }

    // ---------------------------------------------------------------- memory

    fn read_memory(&mut self, target: &Target, payload: &str) -> String {
        let Some((address, length)) = payload.split_once(',') else {
            return "E01".to_string();
        };
        let (Some(address), Some(length)) = (rsp::parse_number(address), rsp::parse_number(length))
        else {
            return "E01".to_string();
        };
        let length = (length as usize).min(PACKET_SIZE) as u32;
        match target.read_memory(address as u32, length) {
            Ok(data) => rsp::to_hex(&data),
            Err(message) => {
                self.note(&message);
                "E01".to_string()
            }
        }
    }

    fn write_memory(&mut self, target: &mut Target, payload: &str) -> String {
        let Some((header, data)) = payload.split_once(':') else {
            return "E01".to_string();
        };
        let Some((address, length)) = header.split_once(',') else {
            return "E01".to_string();
        };
        let (Some(address), Some(length)) = (rsp::parse_number(address), rsp::parse_number(length))
        else {
            return "E01".to_string();
        };
        let Some(bytes) = rsp::from_hex(data.as_bytes()) else {
            return "E01".to_string();
        };
        if bytes.len() != length as usize {
            return "E01".to_string();
        }
        match target.write_memory(address as u32, &bytes) {
            Ok(()) => "OK".to_string(),
            Err(message) => {
                self.note(&message);
                "E01".to_string()
            }
        }
    }

    fn binary_write(&mut self, target: &mut Target, payload: &[u8]) -> Reply {
        let Some(colon) = payload.iter().position(|byte| *byte == b':') else {
            return Reply::Text("E01".to_string());
        };
        let header = String::from_utf8_lossy(&payload[..colon]);
        let Some((address, length)) = header.split_once(',') else {
            return Reply::Text("E01".to_string());
        };
        let (Some(address), Some(length)) = (rsp::parse_number(address), rsp::parse_number(length))
        else {
            return Reply::Text("E01".to_string());
        };
        let data = &payload[colon + 1..];
        let length = length as usize;
        if data.len() < length {
            return Reply::Text("E01".to_string());
        }
        match target.write_memory(address as u32, &data[..length]) {
            Ok(()) => Reply::Text("OK".to_string()),
            Err(message) => {
                self.note(&message);
                Reply::Text("E01".to_string())
            }
        }
    }

    fn flash_write(&mut self, target: &mut Target, payload: &[u8]) -> Reply {
        let Some(colon) = payload.iter().position(|byte| *byte == b':') else {
            return Reply::Text("E01".to_string());
        };
        let address = String::from_utf8_lossy(&payload[..colon]);
        let Some(address) = rsp::parse_number(&address) else {
            return Reply::Text("E01".to_string());
        };
        match target.flash_write(address as u32, &payload[colon + 1..]) {
            Ok(()) => Reply::Text("OK".to_string()),
            Err(message) => {
                eprintln!("error: {message}");
                Reply::Text("E01".to_string())
            }
        }
    }

    // ---------------------------------------------------------- breakpoints

    fn insert_point(&mut self, target: &mut Target, payload: &str) -> String {
        let Some((kind, address, size)) = parse_point(payload) else {
            return "E01".to_string();
        };
        let inserted = match kind {
            PointKind::Software | PointKind::Hardware => {
                target.insert_breakpoint(kind, address, size)
            }
            PointKind::Write | PointKind::Read | PointKind::Access => {
                target.insert_watchpoint(kind, address, size)
            }
        };
        if inserted {
            "OK".to_string()
        } else {
            "E01".to_string()
        }
    }

    fn remove_point(&mut self, target: &mut Target, payload: &str) -> String {
        let Some((kind, address, size)) = parse_point(payload) else {
            return "E01".to_string();
        };
        let removed = match kind {
            PointKind::Software | PointKind::Hardware => {
                target.remove_breakpoint(kind, address, size)
            }
            PointKind::Write | PointKind::Read | PointKind::Access => {
                target.remove_watchpoint(address, size)
            }
        };
        if removed {
            "OK".to_string()
        } else {
            "E01".to_string()
        }
    }

    // -------------------------------------------------------------- queries

    fn query(
        &mut self,
        connection: &mut Connection,
        target: &mut Target,
        text: &str,
    ) -> io::Result<Reply> {
        if text.starts_with("qSupported") {
            return Ok(Reply::Text(format!(
                "PacketSize={PACKET_SIZE:x};qXfer:features:read+;\
                 qXfer:memory-map:read+;qXfer:threads:read+;QStartNoAckMode+;\
                 vContSupported+;swbreak+;hwbreak+"
            )));
        }
        if let Some(request) = text.strip_prefix("qXfer:") {
            return Ok(Reply::Text(self.xfer(request)));
        }
        if let Some(command) = text.strip_prefix("qRcmd,") {
            return match rsp::from_hex(command.as_bytes()) {
                Some(bytes) => self.monitor(connection, target, &String::from_utf8_lossy(&bytes)),
                None => Ok(Reply::Text("E01".to_string())),
            };
        }
        if text.starts_with("qAttached") {
            return Ok(Reply::Text("1".to_string()));
        }
        if text.starts_with("qThreadExtraInfo") {
            return Ok(Reply::Text(rsp::to_hex(b"Cortex-M3")));
        }
        if text.starts_with("qSymbol") {
            return Ok(Reply::Text("OK".to_string()));
        }
        Ok(match text {
            "qC" => Reply::Text("QC1".to_string()),
            "qfThreadInfo" => Reply::Text("m1".to_string()),
            "qsThreadInfo" => Reply::Text("l".to_string()),
            "qOffsets" => Reply::Text("Text=0;Data=0;Bss=0".to_string()),
            "qTStatus" | "qTfV" | "qTsV" => Reply::Text(String::new()),
            _ => Reply::None,
        })
    }

    fn set_query(&mut self, connection: &mut Connection, text: &str) -> Reply {
        if text.starts_with("QStartNoAckMode") {
            // The reply is sent with acknowledgements still on; the mode starts
            // with the next packet.
            connection.set_acknowledge(false);
            self.note("no-acknowledgement mode");
            return Reply::Text("OK".to_string());
        }
        Reply::None
    }

    fn extended(
        &mut self,
        connection: &mut Connection,
        target: &mut Target,
        text: &str,
    ) -> io::Result<Reply> {
        if text == "vMustReplyEmpty" {
            return Ok(Reply::Text(String::new()));
        }
        if text == "vCont?" {
            return Ok(Reply::Text("vCont;c;C;s;S;t".to_string()));
        }
        if let Some(action) = text.strip_prefix("vCont;") {
            let action = action.split(':').next().unwrap_or("");
            return match action {
                "c" => self.continue_target(connection, target),
                "s" => self.step_target(target),
                "t" => {
                    target.halt();
                    Ok(Reply::Text(self.stop_reply(Some(&Stop::Halt))))
                }
                _ => Ok(Reply::None),
            };
        }
        if let Some(range) = text.strip_prefix("vFlashErase:") {
            let Some((address, length)) = range.split_once(',') else {
                return Ok(Reply::Text("E01".to_string()));
            };
            let (Some(address), Some(length)) =
                (rsp::parse_number(address), rsp::parse_number(length))
            else {
                return Ok(Reply::Text("E01".to_string()));
            };
            return Ok(match target.flash_erase(address as u32, length as u32) {
                Ok(()) => Reply::Text("OK".to_string()),
                Err(message) => {
                    eprintln!("error: {message}");
                    Reply::Text("E01".to_string())
                }
            });
        }
        if text.starts_with("vFlashDone") {
            return Ok(Reply::Text("OK".to_string()));
        }
        if text.starts_with("vKill") {
            // GDB kills the inferior before `run`; keeping the connection open
            // lets it start the firmware again without reconnecting.
            self.note("inferior killed, connection kept open");
            return Ok(Reply::Text("OK".to_string()));
        }
        if text.starts_with("vRun") {
            return Ok(match target.hard_reset() {
                Ok(()) => Reply::Text(self.stop_reply(None)),
                Err(message) => {
                    self.note(&message);
                    Reply::Text("E01".to_string())
                }
            });
        }
        Ok(Reply::None)
    }

    /// Serves `qXfer` reads: only `read` is supported, in `offset,length` chunks.
    fn xfer(&self, request: &str) -> String {
        let Some((object, range)) = request.rsplit_once(':') else {
            return String::new();
        };
        let mut fields = object.splitn(2, ':');
        let name = fields.next().unwrap_or("");
        let operation = fields.next().unwrap_or("");
        if operation != "read" && !operation.starts_with("read:") {
            return String::new();
        }
        let (offset, length) = match range.split_once(',') {
            Some((offset, length)) => (
                rsp::parse_number(offset).unwrap_or(0) as usize,
                rsp::parse_number(length).unwrap_or(0) as usize,
            ),
            None => (0, PACKET_SIZE),
        };
        let data: &[u8] = match name {
            "features" => description::TARGET_XML.as_bytes(),
            "memory-map" => self.memory_map.as_bytes(),
            "threads" => THREADS_XML.as_bytes(),
            _ => return String::new(),
        };

        let start = offset.min(data.len());
        let end = start.saturating_add(length).min(data.len());
        let more = if end < data.len() { 'm' } else { 'l' };
        format!("{more}{}", String::from_utf8_lossy(&data[start..end]))
    }

    fn monitor(
        &mut self,
        connection: &mut Connection,
        target: &mut Target,
        command: &str,
    ) -> io::Result<Reply> {
        let output = target.monitor(command);
        for line in output.lines() {
            let line = format!("{line}\n");
            connection.send(&format!("O{}", rsp::to_hex(line.as_bytes())))?;
        }
        Ok(Reply::Text("OK".to_string()))
    }

    fn note(&self, message: &str) {
        if self.options.verbose {
            eprintln!("[gsim-gdb] {message}");
        }
    }
}

/// Shortens a payload for the verbose log and escapes control bytes.
fn preview(payload: &[u8]) -> String {
    let mut text = String::new();
    for byte in payload.iter().take(96) {
        match byte {
            b'\n' => text.push_str("\\n"),
            b'\r' => text.push_str("\\r"),
            0x20..=0x7E => text.push(*byte as char),
            _ => text.push_str(&format!("\\x{byte:02x}")),
        }
    }
    if payload.len() > 96 {
        text.push_str("...");
    }
    text
}

/// Parses the `<type>,<address>,<size>` part of a `Z`/`z` packet.
fn parse_point(payload: &str) -> Option<(PointKind, u32, u32)> {    let mut fields = payload.split(',');
    let kind = match fields.next()? {
        "0" => PointKind::Software,
        "1" => PointKind::Hardware,
        "2" => PointKind::Write,
        "3" => PointKind::Read,
        "4" => PointKind::Access,
        _ => return None,
    };
    let address = rsp::parse_number(fields.next()?)?;
    let size = fields.next().map_or(Some(0), rsp::parse_number)?;
    Some((kind, address as u32, size as u32))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpStream;

    fn server() -> (TcpListener, GdbServer) {
        GdbServer::bind(ServerOptions {
            bind: SocketAddr::from(([127, 0, 0, 1], 0)),
            echo_uart: false,
            ..ServerOptions::default()
        })
        .expect("the loopback socket binds")
    }

    /// Sends a raw request and returns the decoded reply payload.
    fn request(stream: &mut TcpStream, buffer: &mut Vec<u8>, payload: &str) -> String {
        use std::io::{Read, Write};
        stream
            .write_all(&rsp::encode(payload))
            .expect("the request is written");
        loop {
            match rsp::next_frame(buffer) {
                rsp::Frame::Packet(reply) => {
                    return String::from_utf8_lossy(&reply).to_string();
                }
                rsp::Frame::Incomplete => {
                    let mut chunk = [0u8; 4096];
                    let read = stream.read(&mut chunk).expect("the reply is read");
                    assert!(read > 0, "the server closed the connection");
                    buffer.extend_from_slice(&chunk[..read]);
                }
                other => panic!("unexpected frame {other:?}"),
            }
        }
    }

    #[test]
    fn the_server_reports_the_supported_features() {
        let (listener, server) = server();
        let address = listener.local_addr().expect("a bound address");
        std::thread::spawn(move || {
            let _ = server.serve(listener);
        });

        let mut stream = TcpStream::connect(address).expect("the server accepts connections");
        let mut buffer = Vec::new();
        let reply = request(&mut stream, &mut buffer, "qSupported:multiprocess+");
        assert!(reply.contains("qXfer:features:read+"), "{reply}");
        assert!(reply.contains("qXfer:memory-map:read+"), "{reply}");
        assert!(reply.contains("swbreak+"), "{reply}");

        let reply = request(&mut stream, &mut buffer, "?");
        assert!(reply.starts_with("T05"), "{reply}");

        let registers = request(&mut stream, &mut buffer, "g");
        assert_eq!(registers.len(), description::REGISTER_COUNT * 8);

        let reply = request(&mut stream, &mut buffer, "qXfer:features:read:target.xml:0,3ff");
        assert!(reply.starts_with('l') || reply.starts_with('m'), "{reply}");
        assert!(reply.contains("<target"), "{reply}");
    }

    #[test]
    fn writing_ram_round_trips_through_the_bus() {
        let (listener, server) = server();
        let address = listener.local_addr().expect("a bound address");
        std::thread::spawn(move || {
            let _ = server.serve(listener);
        });

        let mut stream = TcpStream::connect(address).expect("the server accepts connections");
        let mut buffer = Vec::new();
        request(&mut stream, &mut buffer, "qSupported");
        let reply = request(&mut stream, &mut buffer, "M20000000,4:78563412");
        assert_eq!(reply, "OK");
        let reply = request(&mut stream, &mut buffer, "m20000000,4");
        assert_eq!(reply, "78563412");
    }
}
