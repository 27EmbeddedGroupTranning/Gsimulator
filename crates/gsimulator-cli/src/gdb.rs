//! `gsimulator gdb` – serve a virtual ST-Link so GDB can flash and debug.
//!
//! The server speaks the GDB remote serial protocol on the port OpenOCD uses,
//! so the usual `target extended-remote :3333`, `load`, `break` and `continue`
//! work against the simulator exactly as they do against a real board.

use std::net::SocketAddr;
use std::path::PathBuf;

use gsim_gdb::{GdbServer, ServerOptions};

/// Options of the `gdb` sub command.
#[derive(Clone, Debug)]
pub struct GdbOptions {
    /// Image programmed before the first connection, if any.
    pub elf: Option<PathBuf>,
    /// Address the server listens on.
    pub bind: SocketAddr,
    /// Crystal on `OSC_IN`.
    pub hse_hz: u32,
    /// Copy the firmware's UART output to stdout.
    pub echo_uart: bool,
    /// Log protocol activity on stderr.
    pub verbose: bool,
}

impl Default for GdbOptions {
    fn default() -> Self {
        GdbOptions {
            elf: None,
            bind: SocketAddr::from(([127, 0, 0, 1], 3333)),
            hse_hz: 8_000_000,
            echo_uart: true,
            verbose: false,
        }
    }
}

/// Binds the port and serves debuggers until the process is stopped.
pub fn run(options: GdbOptions) -> Result<(), String> {
    let server = ServerOptions {
        bind: options.bind,
        elf: options.elf.clone(),
        hse_hz: options.hse_hz,
        echo_uart: options.echo_uart,
        verbose: options.verbose,
    };
    let (listener, server) = GdbServer::bind(server)
        .map_err(|error| format!("cannot listen on {}: {error}", options.bind))?;
    eprintln!("press Ctrl+C to stop the server");
    server
        .serve(listener)
        .map_err(|error| format!("the debug server stopped: {error}"))
}
