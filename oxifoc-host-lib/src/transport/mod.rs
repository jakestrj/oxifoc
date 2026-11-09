//! Transport abstraction for oxifoc host communication over CAN / Virtual CAN (`vcan`).
//!
//! Uses `streaming-protocol` + `nmea` fast-packet framing over CAN:
//! - UDP Multicast Virtual CAN Bus (default on macOS/Windows, and automatic fallback on Linux)
//! - Linux SocketCAN (`vcan0`, `can0`, etc.)

pub mod can;

use serde::Deserialize;

/// Transport type selection.
#[derive(Debug, Clone, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TransportType {
    /// CAN / Virtual CAN (`vcan0` / `can0` — UDP Multicast Virtual CAN on macOS, SocketCAN on Linux)
    #[default]
    #[serde(alias = "vcan", alias = "socketcan")]
    Can,
}

/// Configuration for transport selection.
#[derive(Debug, Clone)]
pub enum TransportConfig {
    Can { interface: String },
}
