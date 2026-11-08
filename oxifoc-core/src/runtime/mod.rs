//! Async runtime for motor control with `streaming-protocol` + `nmea` over CAN.
//!
//! This module provides async telemetry publishers, watch channels, and
//! `streaming-protocol` + `nmea` request/response handlers.

pub mod servers;
pub mod streaming;

/// I/O wrappers for embedded-io-async (requires `embassy-hal` feature)
#[cfg(feature = "embassy-hal")]
pub mod io;

pub use servers::*;
pub use streaming::*;
