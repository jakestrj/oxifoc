//! CAN / Virtual CAN (`vcan`) / PCAN-USB transport for `oxifoc-host-lib`.
//!
//! Supports three backends transparently:
//! 1. **UDP Multicast Virtual CAN Bus** (default for `vcan*` / `mcast:*` on macOS/Windows,
//!    and automatic fallback on Linux when kernel `vcan` is unavailable):
//!    - Joins the RFC 2365 local-scope multicast group `239.255.79.88` on a port
//!      derived deterministically from the interface name (`vcan0` -> `29536`,
//!      `vcan1` -> `29537`, etc.) with `SO_REUSEADDR` + `SO_REUSEPORT` and
//!      multicast loopback enabled on `127.0.0.1`.
//!    - Each datagram carries a 10-byte classic CAN frame (`CAN_WIRE_FRAME_LEN`):
//!      2-byte little-endian 11-bit arbitration ID + 8-byte NMEA fast-packet payload.
//! 2. **Linux SocketCAN** (`#[cfg(target_os = "linux")]`):
//!    - Uses native kernel `AF_CAN` sockets (`vcan0`, `can0`, etc.) when present.
//! 3. **PEAK-System PCAN-USB** (`#[cfg(feature = "pcan")]`):
//!    - Uses `pcbusb` (`MacCAN` `libPCBUSB.dylib` on macOS, `PCAN-Basic` elsewhere)
//!      for `pcan0` / `PCAN_USBBUS1` hardware interfaces without falling back to multicast.

use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use oxifoc_core::icd::{
    CAN_BITRATE, CAN_WIRE_FRAME_LEN, NMEA_INTER_FRAME_DELAY_MS, NMEA_RX_ARBITRATION_ID,
    VCAN_MULTICAST_IP, vcan_multicast_port,
};
use oxifoc_core::proto::oxifoc::WireMessage;
use oxifoc_core::runtime::servers::encode_wire_message_frames;
use tokio::net::UdpSocket;
use tracing::info;

use super::pcan;

/// Resolve a CAN interface name (e.g. `"vcan0"`, `"mcast:vcan0"`, or
/// `"239.255.79.88:29536"`) to its UDP Multicast Virtual CAN socket address.
#[must_use]
pub fn multicast_addr_for_interface(can_interface: &str) -> SocketAddrV4 {
    let trimmed = can_interface
        .strip_prefix("mcast:")
        .unwrap_or(can_interface);
    if let Ok(addr) = trimmed.parse::<SocketAddrV4>() {
        return addr;
    }
    let ip = Ipv4Addr::from(VCAN_MULTICAST_IP);
    let port = vcan_multicast_port(trimmed);
    SocketAddrV4::new(ip, port)
}

/// Bind a non-blocking UDP socket configured for the local multicast virtual CAN bus.
pub fn bind_multicast_can_socket(can_interface: &str) -> Result<(UdpSocket, SocketAddr)> {
    let mcast_addr = multicast_addr_for_interface(can_interface);
    let sock = socket2::Socket::new(
        socket2::Domain::IPV4,
        socket2::Type::DGRAM,
        Some(socket2::Protocol::UDP),
    )
    .context("create UDP multicast socket")?;

    sock.set_reuse_address(true)
        .context("set SO_REUSEADDR on virtual CAN socket")?;
    #[cfg(unix)]
    sock.set_reuse_port(true)
        .context("set SO_REUSEPORT on virtual CAN socket")?;

    sock.set_multicast_loop_v4(true)
        .context("enable IP_MULTICAST_LOOP on virtual CAN socket")?;
    sock.set_multicast_if_v4(&Ipv4Addr::LOCALHOST)
        .context("set IP_MULTICAST_IF to 127.0.0.1")?;

    let bind_addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, mcast_addr.port());
    sock.bind(&bind_addr.into())
        .with_context(|| format!("bind virtual CAN socket to {bind_addr}"))?;

    sock.join_multicast_v4(mcast_addr.ip(), &Ipv4Addr::LOCALHOST)
        .with_context(|| format!("join multicast group {} on 127.0.0.1", mcast_addr.ip()))?;

    sock.set_nonblocking(true)
        .context("set virtual CAN socket non-blocking")?;

    let udp = UdpSocket::from_std(sock.into()).context("wrap virtual CAN socket in Tokio")?;
    Ok((udp, SocketAddr::V4(mcast_addr)))
}

/// Connected CAN interface handle (UDP Multicast Virtual CAN, Linux SocketCAN, or PCAN-USB).
#[derive(Clone)]
pub enum CanEndpoint {
    Multicast {
        socket: Arc<UdpSocket>,
        mcast_addr: SocketAddr,
    },
    #[cfg(target_os = "linux")]
    SocketCan {
        socket: Arc<socketcan::tokio::CanSocket>,
    },
    #[cfg(feature = "pcan")]
    Pcan(pcan::PcanEndpoint),
}

/// Open a PEAK PCAN-USB endpoint explicitly (never falls back to UDP multicast).
pub fn connect_pcan(can_interface: &str, bitrate: u32) -> Result<CanEndpoint> {
    pcan::validate_pcan_channel(can_interface)?;
    pcan::validate_pcan_bitrate(bitrate)?;
    pcan::ensure_pcan_feature_enabled()?;

    #[cfg(feature = "pcan")]
    {
        let ep = pcan::PcanEndpoint::open(can_interface, bitrate)?;
        Ok(CanEndpoint::Pcan(ep))
    }

    #[cfg(not(feature = "pcan"))]
    unreachable!("ensure_pcan_feature_enabled returned Err when pcan feature is disabled")
}

/// Open a CAN or Virtual CAN (`vcan`) endpoint by interface name using [`CAN_BITRATE`].
pub fn connect(can_interface: &str) -> Result<CanEndpoint> {
    connect_with_bitrate(can_interface, CAN_BITRATE)
}

/// Open a CAN, Virtual CAN (`vcan`), or PCAN-USB endpoint by interface name and bitrate.
pub fn connect_with_bitrate(can_interface: &str, bitrate: u32) -> Result<CanEndpoint> {
    let force_multicast =
        can_interface.starts_with("mcast:") || can_interface.parse::<SocketAddrV4>().is_ok();

    // Never silently map a PCAN hardware name onto UDP Multicast Virtual CAN.
    if !force_multicast && pcan::is_pcan_interface(can_interface) {
        return connect_pcan(can_interface, bitrate);
    }

    #[cfg(target_os = "linux")]
    {
        if !force_multicast {
            match socketcan::tokio::CanSocket::open(can_interface) {
                Ok(sock) => {
                    info!("Opened Linux SocketCAN interface '{can_interface}'");
                    return Ok(CanEndpoint::SocketCan {
                        socket: Arc::new(sock),
                    });
                }
                Err(e) => {
                    info!(
                        "Kernel SocketCAN interface '{can_interface}' unavailable ({e}); using UDP Multicast Virtual CAN bus"
                    );
                }
            }
        }
    }

    let (udp, mcast_addr) = bind_multicast_can_socket(can_interface)?;
    info!("Joined UDP Multicast Virtual CAN bus '{can_interface}' at {mcast_addr}");
    Ok(CanEndpoint::Multicast {
        socket: Arc::new(udp),
        mcast_addr,
    })
}

impl CanEndpoint {
    async fn send_raw_frame(&self, arb_id: u16, payload: &[u8; 8]) -> Result<()> {
        match self {
            Self::Multicast { socket, mcast_addr } => {
                let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
                wire[0..2].copy_from_slice(&arb_id.to_le_bytes());
                wire[2..10].copy_from_slice(payload);
                socket
                    .send_to(&wire, *mcast_addr)
                    .await
                    .context("send virtual CAN multicast frame")?;
            }
            #[cfg(target_os = "linux")]
            Self::SocketCan { socket } => {
                use socketcan::{CanFrame, EmbeddedFrame, StandardId};
                let std_id = StandardId::new(arb_id)
                    .ok_or_else(|| anyhow::anyhow!("invalid CAN arbitration ID"))?;
                let can_frame = CanFrame::new(std_id, payload)
                    .ok_or_else(|| anyhow::anyhow!("invalid CAN frame payload"))?;
                socket
                    .write_frame(can_frame)
                    .await
                    .context("write SocketCAN frame")?;
            }
            #[cfg(feature = "pcan")]
            Self::Pcan(ep) => {
                ep.send_frame(arb_id, payload).await?;
            }
        }
        Ok(())
    }

    /// Encode a [`WireMessage`] with `streaming-protocol`, fragment it into 8-byte
    /// NMEA fast-packet CAN frames, and transmit each frame on `0x7E5`.
    pub async fn send_wire_message(&self, msg: &WireMessage<'_>) -> Result<()> {
        let mut nmea_msg = encode_wire_message_frames(msg)
            .map_err(|e| anyhow::anyhow!("encode wire message failed: {e:?}"))?;

        while let Some(frame) = nmea_msg.pop_frame() {
            self.send_raw_frame(NMEA_RX_ARBITRATION_ID, &frame.bytes)
                .await?;
            if nmea_msg.num_frames > 1 {
                tokio::time::sleep(Duration::from_millis(NMEA_INTER_FRAME_DELAY_MS)).await;
            }
        }
        Ok(())
    }

    /// Receive the next 8-byte CAN data frame and its 11-bit arbitration ID.
    pub async fn recv_frame(&self) -> Result<(u16, [u8; 8])> {
        match self {
            Self::Multicast { socket, .. } => {
                let mut buf = [0u8; 64];
                loop {
                    let (len, _) = socket
                        .recv_from(&mut buf)
                        .await
                        .context("recv virtual CAN multicast frame")?;
                    if len == CAN_WIRE_FRAME_LEN {
                        let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
                        let mut data = [0u8; 8];
                        data.copy_from_slice(&buf[2..10]);
                        return Ok((arb_id, data));
                    }
                }
            }
            #[cfg(target_os = "linux")]
            Self::SocketCan { socket } => {
                use socketcan::{CanFrame, EmbeddedFrame, Id};
                loop {
                    match socket.read_frame().await.context("read SocketCAN frame")? {
                        CanFrame::Data(frame) => {
                            if let Id::Standard(std_id) = frame.id()
                                && let Ok(data) = frame.data().try_into()
                            {
                                return Ok((std_id.as_raw(), data));
                            }
                        }
                        CanFrame::Remote(_) | CanFrame::Error(_) => {}
                    }
                }
            }
            #[cfg(feature = "pcan")]
            Self::Pcan(ep) => ep.recv_frame().await,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multicast_addr_parses_explicit_and_named_interfaces() {
        let named = multicast_addr_for_interface("vcan0");
        assert_eq!(named.ip(), &Ipv4Addr::from(VCAN_MULTICAST_IP));
        assert_eq!(named.port(), vcan_multicast_port("vcan0"));

        let prefixed = multicast_addr_for_interface("mcast:vcan0");
        assert_eq!(prefixed, named);

        let explicit = multicast_addr_for_interface("239.255.79.88:29999");
        assert_eq!(explicit.port(), 29999);
    }

    #[cfg(not(feature = "pcan"))]
    #[test]
    fn pcan_interface_does_not_silently_fallback_to_multicast_when_feature_disabled() {
        let Err(err) = connect_with_bitrate("pcan0", CAN_BITRATE) else {
            panic!("pcan0 must not fall back to UDP multicast");
        };
        assert!(err.to_string().contains("--features pcan"));
    }
}
