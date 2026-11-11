//! Transport abstraction for oxifoc host communication over CAN / Virtual CAN (`vcan`)
//! and PEAK-System PCAN-USB (`pcbusb`).
//!
//! Uses `streaming-protocol` + `nmea` fast-packet framing over CAN:
//! - UDP Multicast Virtual CAN Bus (default on macOS/Windows, and automatic fallback on Linux)
//! - Linux SocketCAN (`vcan0`, `can0`, etc.)
//! - PEAK-System PCAN-USB (`pcan0` / `PCAN_USBBUS1` via `pcbusb`, behind the `pcan` Cargo feature)

pub mod can;
pub mod pcan;

use serde::Deserialize;

/// Transport type selection.
#[derive(Debug, Clone, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TransportType {
    /// CAN / Virtual CAN (`vcan0` / `can0` — UDP Multicast Virtual CAN on macOS, SocketCAN on Linux,
    /// or PCAN-USB when a `pcan*` / `PCAN_USBBUS*` interface name is specified)
    #[default]
    #[serde(alias = "vcan", alias = "socketcan")]
    Can,
    /// PEAK-System PCAN-USB adapter (`pcan0` / `PCAN_USBBUS1` via `pcbusb` + MacCAN `libPCBUSB`)
    #[serde(alias = "pcbusb")]
    Pcan,
}

/// Configuration for transport selection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransportConfig {
    Can { interface: String, bitrate: u32 },
    Pcan { interface: String, bitrate: u32 },
}
