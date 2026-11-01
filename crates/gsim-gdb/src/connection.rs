//! TCP transport of the remote protocol.
//!
//! The connection keeps the receive buffer, so a packet that arrives while the
//! target is running is queued instead of being lost, and a control-C is
//! recognised without breaking the framing.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::TcpStream;

use crate::rsp::{self, Frame};

/// Result of blocking for the next element of the receive stream.
#[derive(Debug, PartialEq, Eq)]
pub enum Waiting {
    /// A complete packet arrived.
    Packet(Vec<u8>),
    /// The peer asked to interrupt the running target.
    Interrupt,
    /// The peer closed the connection.
    Closed,
}

/// Result of a non-blocking poll while the target runs.
#[derive(Debug, PartialEq, Eq)]
pub enum Poll {
    /// Nothing arrived.
    Idle,
    /// A control-C arrived.
    Interrupt,
    /// The peer closed the connection.
    Closed,
}

/// A GDB connection plus its receive buffer.
pub struct Connection {
    stream: TcpStream,
    buffer: Vec<u8>,
    queued: VecDeque<Vec<u8>>,
    acknowledge: bool,
    non_blocking: bool,
}

impl Connection {
    /// Wraps an accepted socket.
    pub fn new(stream: TcpStream) -> io::Result<Self> {
        stream.set_nodelay(true)?;
        Ok(Connection {
            stream,
            buffer: Vec::new(),
            queued: VecDeque::new(),
            acknowledge: true,
            non_blocking: false,
        })
    }

    /// Turns acknowledgements off after `QStartNoAckMode`.
    pub fn set_acknowledge(&mut self, acknowledge: bool) {
        self.acknowledge = acknowledge;
    }

    /// Keeps the socket non-blocking for the lifetime of the connection.
    ///
    /// A front end that serves the protocol from its UI thread needs this: a
    /// debugger that stops reading must fail the write instead of freezing the
    /// window.
    pub fn set_non_blocking(&mut self, non_blocking: bool) -> io::Result<()> {
        self.non_blocking = non_blocking;
        self.stream.set_nonblocking(non_blocking)
    }

    /// Sends one packet.
    pub fn send(&mut self, payload: &str) -> io::Result<()> {
        self.stream.write_all(&rsp::encode(payload))?;
        self.stream.flush()
    }

    /// Blocks until a packet, an interrupt or the end of the stream arrives.
    pub fn wait(&mut self) -> io::Result<Waiting> {
        loop {
            if let Some(packet) = self.queued.pop_front() {
                self.acknowledge_arrival()?;
                return Ok(Waiting::Packet(packet));
            }
            match rsp::next_frame(&mut self.buffer) {
                Frame::Packet(payload) => {
                    self.acknowledge_arrival()?;
                    return Ok(Waiting::Packet(payload));
                }
                Frame::Interrupt => return Ok(Waiting::Interrupt),
                Frame::BadChecksum => {
                    if self.acknowledge {
                        self.stream.write_all(b"-")?;
                        self.stream.flush()?;
                    }
                }
                Frame::Incomplete => {
                    let mut chunk = [0u8; 1024];
                    let read = self.stream.read(&mut chunk)?;
                    if read == 0 {
                        return Ok(Waiting::Closed);
                    }
                    self.buffer.extend_from_slice(&chunk[..read]);
                }
            }
        }
    }

    /// Waits for data without blocking, used between resume slices.
    ///
    /// Packets that arrive meanwhile are queued for the next [`Connection::wait`].
    pub fn poll(&mut self) -> io::Result<Poll> {
        self.stream.set_nonblocking(true)?;
        let mut closed = false;
        loop {
            let mut chunk = [0u8; 1024];
            match self.stream.read(&mut chunk) {
                Ok(0) => {
                    closed = true;
                    break;
                }
                Ok(read) => self.buffer.extend_from_slice(&chunk[..read]),
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
                Err(error) => {
                    self.restore_blocking()?;
                    return Err(error);
                }
            }
        }
        self.restore_blocking()?;
        if closed {
            return Ok(Poll::Closed);
        }

        loop {
            match rsp::next_frame(&mut self.buffer) {
                Frame::Packet(payload) => self.queued.push_back(payload),
                Frame::Interrupt => return Ok(Poll::Interrupt),
                Frame::BadChecksum | Frame::Incomplete => break,
            }
        }
        Ok(Poll::Idle)
    }

    /// Non-blocking [`Connection::wait`], for a front end that owns the device
    /// on its own thread and only polls the socket once per frame.
    ///
    /// `None` means nothing arrived yet.
    pub fn poll_wait(&mut self) -> io::Result<Option<Waiting>> {
        match self.poll()? {
            Poll::Interrupt => return Ok(Some(Waiting::Interrupt)),
            Poll::Closed => return Ok(Some(Waiting::Closed)),
            Poll::Idle => {}
        }
        match self.queued.pop_front() {
            Some(packet) => {
                self.acknowledge_arrival()?;
                Ok(Some(Waiting::Packet(packet)))
            }
            None => Ok(None),
        }
    }

    /// Closes the socket; called when GDB detaches or kills the session.
    pub fn close(&mut self) {
        let _ = self.stream.shutdown(std::net::Shutdown::Both);
    }

    /// Leaves the socket non-blocking when the owner asked for that.
    fn restore_blocking(&mut self) -> io::Result<()> {
        if self.non_blocking {
            return Ok(());
        }
        self.stream.set_nonblocking(false)
    }

    fn acknowledge_arrival(&mut self) -> io::Result<()> {
        if self.acknowledge {
            self.stream.write_all(b"+")?;
            self.stream.flush()?;
        }
        Ok(())
    }
}
