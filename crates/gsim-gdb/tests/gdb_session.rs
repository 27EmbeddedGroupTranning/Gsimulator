//! Drives the virtual ST-Link the way GDB does, over a real TCP socket.
//!
//! The firmware here is assembled by hand (a few Thumb instructions plus a
//! literal) so the test needs no cross compiler: the point is the protocol and
//! the debug behaviour, not the toolchain.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::time::Duration;

use gsim_gdb::rsp::{self, Frame};
use gsim_gdb::{GdbServer, ServerOptions};

/// Initial stack pointer, reset handler and a small program.
///
/// ```text
/// 0x08000008  movs r0, #1        ; 0x2001
/// 0x0800000a  ldr  r1, [pc, #8]  ; 0x4902  -> literal at 0x08000014
/// 0x0800000c  str  r0, [r1]      ; 0x6008  writes 1 to 0x20000000
/// 0x0800000e  b    .             ; 0xe7fe  endless loop
/// ```
const VECTORS: [u8; 8] = [0x00, 0x50, 0x00, 0x20, 0x09, 0x00, 0x00, 0x08];
const CODE: [u8; 16] = [
    0x01, 0x20, 0x02, 0x49, 0x08, 0x60, 0xFE, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
];

const ENTRY: u32 = 0x0800_0008;
const LOOP: u32 = 0x0800_000E;

/// A minimal GDB client for the checks below.
struct Client {
    stream: TcpStream,
    buffer: Vec<u8>,
}

impl Client {
    fn connect(address: SocketAddr) -> Client {
        let stream = TcpStream::connect(address).expect("the server accepts the connection");
        stream
            .set_read_timeout(Some(Duration::from_secs(30)))
            .expect("a read timeout can be set");
        Client {
            stream,
            buffer: Vec::new(),
        }
    }

    fn send(&mut self, payload: &str) {
        self.stream
            .write_all(&rsp::encode(payload))
            .expect("the request is sent");
    }

    /// Sends a packet whose payload contains binary bytes.
    fn send_bytes(&mut self, payload: &[u8]) {
        let mut wire = Vec::new();
        wire.push(b'$');
        for byte in payload {
            match byte {
                b'#' | b'$' | b'}' | b'*' => {
                    wire.push(b'}');
                    wire.push(byte ^ 0x20);
                }
                _ => wire.push(*byte),
            }
        }
        wire.push(b'#');
        let sum = rsp::checksum(payload);
        wire.extend_from_slice(format!("{sum:02x}").as_bytes());
        self.stream.write_all(&wire).expect("the request is sent");
    }

    fn receive(&mut self) -> String {
        loop {
            match rsp::next_frame(&mut self.buffer) {
                Frame::Packet(payload) => {
                    // Acknowledge like GDB; the server ignores these.
                    self.stream.write_all(b"+").expect("the ack is sent");
                    return String::from_utf8_lossy(&payload).to_string();
                }
                Frame::Interrupt => continue,
                Frame::BadChecksum => panic!("the server sent a corrupt packet"),
                Frame::Incomplete => {
                    let mut chunk = [0u8; 4096];
                    let read = self.stream.read(&mut chunk).expect("the reply is read");
                    assert!(read > 0, "the server closed the connection");
                    self.buffer.extend_from_slice(&chunk[..read]);
                }
            }
        }
    }

    fn request(&mut self, payload: &str) -> String {
        self.send(payload);
        self.receive()
    }

    /// Runs a monitor command and returns the console text it produced.
    fn monitor(&mut self, command: &str) -> String {
        self.send(&format!("qRcmd,{}", rsp::to_hex(command.as_bytes())));
        let mut text = String::new();
        loop {
            let reply = self.receive();
            // `O` packets carry console text as hex; `OK` ends the command.
            match reply.strip_prefix('O') {
                Some(hex)
                    if !hex.is_empty()
                        && hex.len() % 2 == 0
                        && hex.bytes().all(|byte| byte.is_ascii_hexdigit()) =>
                {
                    let bytes = rsp::from_hex(hex.as_bytes()).expect("hex text");
                    text.push_str(&String::from_utf8_lossy(&bytes));
                }
                _ => {
                    assert_eq!(reply, "OK", "unexpected monitor reply");
                    return text;
                }
            }
        }
    }

    fn interrupt(&mut self) {
        self.stream.write_all(&[0x03]).expect("control-C is sent");
    }

    /// Register value from a `g` packet.
    fn register(&mut self, index: usize) -> u32 {
        let reply = self.request("g");
        let words = rsp::words_from_hex(reply.as_bytes()).expect("a register packet");
        words[index]
    }

    /// Programs flash the way GDB's `vFlashWrite` does.
    fn flash_write(&mut self, address: u32, data: &[u8]) {
        let mut payload = format!("vFlashWrite:{address:08x}:").into_bytes();
        payload.extend_from_slice(data);
        self.send_bytes(&payload);
        assert_eq!(self.receive(), "OK", "the flash write is accepted");
    }
}

fn start_server() -> SocketAddr {
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
    address
}

#[test]
fn gdb_can_load_and_debug_the_simulated_flash() {
    let address = start_server();
    let mut client = Client::connect(address);

    let reply = client.request("qSupported:multiprocess+;swbreak+;hwbreak+");
    assert!(reply.contains("qXfer:memory-map:read+"), "{reply}");
    assert!(reply.contains("swbreak+"), "{reply}");

    assert_eq!(client.request("vFlashErase:08000000,400"), "OK");
    client.flash_write(0x0800_0000, &VECTORS);
    client.flash_write(0x0800_0008, &CODE);
    assert_eq!(client.request("vFlashDone"), "OK");

    let mut expected = VECTORS.to_vec();
    expected.extend_from_slice(&CODE);
    assert_eq!(
        client.request("m8000000,18"),
        rsp::to_hex(&expected).as_str()
    );

    let output = client.monitor("reset");
    assert!(output.contains("reset"), "{output}");
    assert_eq!(client.register(15), ENTRY);
    assert_eq!(client.register(13), 0x2000_5000);

    // A watchpoint stops the firmware on the store to 0x20000000.
    assert_eq!(client.request("Z2,20000000,4"), "OK");
    let reply = client.request("vCont;c");
    assert!(reply.starts_with("T05"), "{reply}");
    assert!(reply.contains("awatch:20000000"), "{reply}");
    assert_eq!(client.register(15), LOOP);
    assert_eq!(client.request("z2,20000000,4"), "OK");

    // A breakpoint on the endless loop stops it there.
    assert_eq!(client.request(&format!("Z0,{LOOP:08x},2")), "OK");
    let reply = client.request("vCont;c");
    assert!(reply.starts_with("T05"), "{reply}");
    assert!(reply.contains("swbreak:"), "{reply}");
    assert_eq!(client.register(15), LOOP);

    // A single step leaves the loop where it was but must not trip the
    // breakpoint again.
    let reply = client.request("vCont;s");
    assert!(reply.starts_with("T05"), "{reply}");
    assert_eq!(client.register(15), LOOP);
    assert_eq!(client.request(&format!("z0,{LOOP:08x},2")), "OK");

    // Control-C stops a running target.
    client.send("vCont;c");
    client.interrupt();
    let reply = client.receive();
    assert!(reply.starts_with("T02"), "{reply}");

    // Registers can be written from GDB.
    assert_eq!(client.request("P0=efbeadde"), "OK");
    assert_eq!(client.register(0), 0xDEAD_BEEF);

    assert_eq!(client.request("D"), "OK");
}

#[test]
fn xfer_and_monitor_serve_the_target_description() {
    let address = start_server();
    let mut client = Client::connect(address);

    let reply = client.request("qXfer:features:read:target.xml:0,1000");
    assert!(reply.starts_with('l'), "{reply}");
    assert!(reply.contains("org.gnu.gdb.arm.m-profile"), "{reply}");

    let reply = client.request("qXfer:memory-map:read::0,1000");
    assert!(reply.contains("blocksize"), "{reply}");

    let output = client.monitor("help");
    assert!(output.contains("reset"), "{output}");
    assert!(output.contains("peripherals"), "{output}");

    let output = client.monitor("info");
    assert!(output.contains("instructions"), "{output}");

    let reply = client.request("qAttached");
    assert_eq!(reply, "1");
    assert_eq!(client.request("qC"), "QC1");
    assert_eq!(client.request("qfThreadInfo"), "m1");
    assert_eq!(client.request("vMustReplyEmpty"), "");
}

#[test]
fn binary_writes_reach_sram_and_flash_programming_is_verified() {
    let address = start_server();
    let mut client = Client::connect(address);

    // `X` packets carry raw bytes instead of hex.
    client.send_bytes(b"X20000000,4:\x01\x02\x03\x04");
    assert_eq!(client.receive(), "OK");
    assert_eq!(client.request("m20000000,4"), "01020304");

    // Flash writes go through the controller: erase, then program.
    assert_eq!(client.request("vFlashErase:08000400,400"), "OK");
    assert_eq!(client.request("M8000400,4:11223344"), "OK");
    assert_eq!(client.request("m8000400,4"), "11223344");

    // Unerased bits cannot be set again, so the second write is reported as a
    // failed verify instead of silently corrupting the image.
    assert_eq!(client.request("M8000400,4:ffffffff"), "E01");
}
