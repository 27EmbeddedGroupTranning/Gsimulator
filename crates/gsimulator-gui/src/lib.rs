//! Gsimulator desktop front end.
//!
//! Library target so the UI code can be exercised headlessly by tests; the
//! `gsimulator-gui` binary is a thin wrapper around [`app::App`].

pub mod app;
pub mod board;
pub mod bridge;
pub mod canbus;
pub mod canvas;
pub mod library;
pub mod peripherals;
pub mod route;
pub mod scope;
pub mod sim;
pub mod stimulus;
pub mod waveform;
