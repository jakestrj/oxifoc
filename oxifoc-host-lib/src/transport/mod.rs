//! Transport abstraction for oxifoc host communication.
//!
//! Provides different transport backends:
//! - CAN / Virtual CAN (`vcan0` via UDP Multicast Virtual CAN on macOS, or Linux SocketCAN)
//! - TCP (10-byte CAN-framed stream for `oxifoc-virtual`)
//! - UDP (10-byte CAN-framed datagrams for `oxifoc-virtual`)
//! - Serial (UART over USB VCP)
//! - RTT (via debug probe using probe-rs)
//! - USB / BLE

pub mod ble;
pub mod can;
#[cfg(feature = "desktop")]
pub mod rtt;
#[cfg(feature = "desktop")]
pub mod serial;
pub mod tcp;
pub mod udp;
pub mod usb;

use serde::Deserialize;
use tokio::io::{AsyncRead, AsyncWrite};

/// Transport type selection.
#[derive(Debug, Clone, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TransportType {
    /// Serial port (UART over USB VCP) - default on desktop
    #[cfg_attr(feature = "desktop", default)]
    #[cfg(feature = "desktop")]
    Serial,
    /// RTT via debug probe (probe-rs)
    #[cfg(feature = "desktop")]
    Rtt,
    /// TCP connection (for oxifoc-virtual or remote devices)
    Tcp,
    /// UDP connection
    Udp,
    /// CAN / Virtual CAN (`vcan0` / `can0` — UDP Multicast Virtual CAN on macOS, SocketCAN on Linux)
    #[serde(alias = "vcan", alias = "socketcan")]
    Can,
    /// USB bulk
    Usb,
    /// BLE (via bluest) - default on Android
    #[cfg_attr(not(feature = "desktop"), default)]
    Ble,
}

/// Configuration for transport selection.
#[derive(Debug, Clone)]
pub enum TransportConfig {
    #[cfg(feature = "desktop")]
    Serial {
        path: String,
        baud: u32,
    },
    #[cfg(feature = "desktop")]
    Rtt {
        probe: Option<String>,
        chip: String,
    },
    Tcp {
        host: String,
        port: u16,
    },
    Udp {
        host: String,
        port: u16,
    },
    Can {
        interface: String,
    },
    Usb,
    Ble {
        device: bluest::Device,
    },
}

/// Stream transport returning AsyncRead/AsyncWrite pairs (TCP, serial, RTT).
pub struct CobsStreamTransport {
    /// Reader for incoming data (device -> host)
    pub reader: Box<dyn AsyncRead + Send + Unpin>,
    /// Writer for outgoing data (host -> device)
    pub writer: Box<dyn AsyncWrite + Send + Unpin>,
    /// Optional reader for defmt data (RTT mode only, channel 0).
    pub defmt_reader: Option<Box<dyn AsyncRead + Send + Unpin>>,
}
