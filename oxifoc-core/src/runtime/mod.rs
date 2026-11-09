//! Async runtime for motor control with `streaming-protocol` + `nmea` over CAN.
//!
//! This module provides async telemetry publishers, watch channels, and
//! `streaming-protocol` + `nmea` request/response handlers.

pub mod servers;
pub mod streaming;

pub use servers::*;
pub use streaming::*;
