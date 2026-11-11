//! Virtual CAN (`vcan0` / `can0`) server transporting `streaming-protocol` + `nmea` frames.
//!
//! Supports two backends transparently:
//! 1. **UDP Multicast Virtual CAN Bus** (default on macOS/Windows, and automatic
//!    fallback on Linux when kernel `vcan` is unavailable or `mcast:<iface>` /
//!    `<multicast_ip>:<port>` is given):
//!    - Joins the RFC 2365 local-scope multicast group `239.255.79.88` on a port
//!      derived deterministically from the interface name (`vcan0` -> `29536`,
//!      `vcan1` -> `29537`, etc.) with `SO_REUSEADDR` + `SO_REUSEPORT` and
//!      multicast loopback enabled on `127.0.0.1`.
//!    - Each datagram carries a 10-byte classic CAN frame (`CAN_WIRE_FRAME_LEN`):
//!      2-byte little-endian 11-bit arbitration ID + 8-byte NMEA fast-packet payload.
//! 2. **Linux SocketCAN** (`#[cfg(target_os = "linux")]`):
//!    - Uses native kernel `AF_CAN` sockets (`vcan0`, `can0`, etc.) when present.
//!
//! Both backends use standard 11-bit CAN arbitration IDs:
//! - `NMEA_RX_ARBITRATION_ID = 0x7E5` (host -> device)
//! - `NMEA_TX_ARBITRATION_ID = 0x7ED` (device -> host)

use core::cell::RefCell;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use critical_section::Mutex as CriticalSectionMutex;
use nmea::nmea_message::{MAX_NMEA_PACKET_SIZE, Message as NmeaMessage};
use oxifoc_core::foc::fault::{FaultRegistry, StandardFault};
use oxifoc_core::icd::{
    CAN_WIRE_FRAME_LEN, LIVENESS_TIMEOUT_MS, NMEA_INTER_FRAME_DELAY_MS, NMEA_RX_ARBITRATION_ID,
    NMEA_TX_ARBITRATION_ID, VCAN_MULTICAST_IP, vcan_multicast_port,
};
use oxifoc_core::runtime::servers::{
    CommsError, MessageSender, NmeaCanReceiver, run_telemetry_publishers, send_messages,
};
use oxifoc_core::runtime::streaming::publish_gate_driver_telemetry;
use oxifoc_core::state::MotorControlState;
use oxifoc_core::types::{GateDriverSwitch, GateDriverSwitchTelemetry};
use tokio::net::UdpSocket;
use tracing::{debug, info, warn};

/// Periodically publish simulated EiceDRIVER gate driver telemetry across all 6 switches (`UH`..`WL`).
async fn run_virtual_gate_driver_publisher() {
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    let mut rr_idx: usize = 0;
    let mut seq: u32 = 0;
    loop {
        interval.tick().await;
        let switch_id = GateDriverSwitch::ALL[rr_idx];
        seq = seq.wrapping_add(1);
        let temp_offset = (rr_idx as f32) * 0.4;
        publish_gate_driver_telemetry(GateDriverSwitchTelemetry {
            switch_id,
            configured: true,
            i2c_ok: true,
            flt_n: true,
            rdyc: true,
            chip_rdy: true,
            pri_rdy: true,
            sec_rdy: true,
            secuvevt: 0,
            gfltevt: GateDriverSwitchTelemetry::GFLTEVT_NO_FAULT,
            fltevt: 0,
            pinstat: 0x05,
            comerrst: 0,
            chipstat: 0x08,
            evtstick: 0,
            uv1fcnt: 0,
            uv2fcnt: 0,
            d2ecnt: 0,
            adcmvdif: 19.5,
            adcmgnd2: 4.5,
            adcmvcc2: 15.0,
            adcmtemp: 33.5 + temp_offset,
            board_temp_c_x10: 310,
            seq,
        });
        rr_idx = (rr_idx + 1) % GateDriverSwitch::COUNT;
    }
}

/// Resolve a CAN interface name (e.g. `"vcan0"`, `"mcast:vcan0"`, or
/// `"239.255.79.88:29536"`) to its UDP Multicast Virtual CAN socket address.
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

struct MulticastCanSender {
    socket: Arc<UdpSocket>,
    mcast_addr: SocketAddr,
}

impl MessageSender for MulticastCanSender {
    async fn send_message_internal(&mut self, encoded_buf: &mut [u8]) -> Result<(), CommsError> {
        if encoded_buf.len() > MAX_NMEA_PACKET_SIZE {
            return Err(CommsError::NmeaError);
        }
        let mut nmea_msg = NmeaMessage::from_payload(encoded_buf, 0);
        let id_bytes = NMEA_TX_ARBITRATION_ID.to_le_bytes();
        while let Some(frame) = nmea_msg.pop_frame() {
            let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
            wire[0..2].copy_from_slice(&id_bytes);
            wire[2..10].copy_from_slice(&frame.bytes);
            self.socket
                .send_to(&wire, self.mcast_addr)
                .await
                .map_err(|_| CommsError::CanError)?;
            if nmea_msg.num_frames > 1 {
                tokio::time::sleep(Duration::from_millis(NMEA_INTER_FRAME_DELAY_MS)).await;
            }
        }
        Ok(())
    }
}

async fn run_multicast(
    can_interface: &str,
    foc_freq_hz: u32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
) -> Result<()> {
    let (udp, mcast_addr) = bind_multicast_can_socket(can_interface)?;
    let socket = Arc::new(udp);
    info!(
        "UDP Multicast Virtual CAN server listening on {can_interface} -> {mcast_addr} (RX=0x7E5, TX=0x7ED)"
    );

    // Spawn persistent background telemetry publishers (slow, fast, fault, and gate driver watches).
    tokio::spawn(
        run_telemetry_publishers::<StandardFault, crate::TokioTimer>(
            state_mutex,
            fault_registry,
            foc_freq_hz,
        ),
    );
    tokio::spawn(run_virtual_gate_driver_publisher());

    let boot_instant = Instant::now();
    let last_rx_ms = Arc::new(AtomicU64::new(0));

    // Outbound CAN TX task (watches telemetry channels and multicast-transmits 0x7ED frames).
    tokio::spawn({
        let socket = socket.clone();
        async move {
            let mut sender = MulticastCanSender { socket, mcast_addr };
            send_messages(&mut sender).await;
        }
    });

    // Liveness monitor task: marks link inactive if no RX frame arrives within LIVENESS_TIMEOUT_MS.
    tokio::spawn({
        let last_rx_ms = last_rx_ms.clone();
        async move {
            let mut interval = tokio::time::interval(Duration::from_millis(100));
            loop {
                interval.tick().await;
                let last = last_rx_ms.load(Ordering::Relaxed);
                if last > 0 {
                    let elapsed = (boot_instant.elapsed().as_millis() as u64).saturating_sub(last);
                    if elapsed > LIVENESS_TIMEOUT_MS {
                        critical_section::with(|cs| {
                            state_mutex.borrow(cs).borrow_mut().set_link_inactive();
                        });
                    }
                }
            }
        }
    });

    // Inbound CAN RX loop (reads 10-byte multicast CAN frames and dispatches 0x7E5 NMEA frames).
    let mut nmea_rx = NmeaCanReceiver::new();
    let mut buf = [0u8; 64];
    loop {
        match socket.recv_from(&mut buf).await {
            Ok((len, src_addr)) => {
                if len != CAN_WIRE_FRAME_LEN {
                    continue;
                }
                let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
                if arb_id == NMEA_RX_ARBITRATION_ID {
                    last_rx_ms.store(boot_instant.elapsed().as_millis() as u64, Ordering::Relaxed);
                    if let Err(e) =
                        nmea_rx.handle_frame(&buf[2..10], state_mutex, fault_registry, foc_freq_hz)
                    {
                        debug!(
                            "Virtual CAN multicast RX decode error on {can_interface} from {src_addr}: {e:?}"
                        );
                    }
                }
            }
            Err(e) => {
                warn!("Virtual CAN multicast read error on {can_interface}: {e}");
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
        }
    }
}

#[cfg(target_os = "linux")]
mod linux_socketcan {
    use super::*;
    use socketcan::tokio::CanSocket;
    use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};

    struct SocketCanSender {
        socket: Arc<CanSocket>,
    }

    impl MessageSender for SocketCanSender {
        async fn send_message_internal(
            &mut self,
            encoded_buf: &mut [u8],
        ) -> Result<(), CommsError> {
            if encoded_buf.len() > MAX_NMEA_PACKET_SIZE {
                return Err(CommsError::NmeaError);
            }
            let Some(std_id) = StandardId::new(NMEA_TX_ARBITRATION_ID) else {
                return Err(CommsError::CanError);
            };
            let mut nmea_msg = NmeaMessage::from_payload(encoded_buf, 0);
            while let Some(frame) = nmea_msg.pop_frame() {
                let Some(can_frame) = CanFrame::new(std_id, &frame.bytes) else {
                    return Err(CommsError::CanError);
                };
                self.socket
                    .write_frame(can_frame)
                    .await
                    .map_err(|_| CommsError::CanError)?;
                if nmea_msg.num_frames > 1 {
                    tokio::time::sleep(Duration::from_millis(NMEA_INTER_FRAME_DELAY_MS)).await;
                }
            }
            Ok(())
        }
    }

    pub fn try_open(can_interface: &str) -> Result<CanSocket> {
        CanSocket::open(can_interface)
            .with_context(|| format!("failed to open SocketCAN interface '{can_interface}'"))
    }

    pub async fn run_with_socket(
        can_interface: &str,
        socket: CanSocket,
        foc_freq_hz: u32,
        state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
        fault_registry: &'static FaultRegistry<StandardFault>,
    ) -> Result<()> {
        let socket = Arc::new(socket);
        info!("SocketCAN server listening on {can_interface} (RX=0x7E5, TX=0x7ED)");

        tokio::spawn(
            run_telemetry_publishers::<StandardFault, crate::TokioTimer>(
                state_mutex,
                fault_registry,
                foc_freq_hz,
            ),
        );
        tokio::spawn(run_virtual_gate_driver_publisher());

        let boot_instant = Instant::now();
        let last_rx_ms = Arc::new(AtomicU64::new(0));

        tokio::spawn({
            let socket = socket.clone();
            async move {
                let mut sender = SocketCanSender { socket };
                send_messages(&mut sender).await;
            }
        });

        tokio::spawn({
            let last_rx_ms = last_rx_ms.clone();
            async move {
                let mut interval = tokio::time::interval(Duration::from_millis(100));
                loop {
                    interval.tick().await;
                    let last = last_rx_ms.load(Ordering::Relaxed);
                    if last > 0 {
                        let elapsed =
                            (boot_instant.elapsed().as_millis() as u64).saturating_sub(last);
                        if elapsed > LIVENESS_TIMEOUT_MS {
                            critical_section::with(|cs| {
                                state_mutex.borrow(cs).borrow_mut().set_link_inactive();
                            });
                        }
                    }
                }
            }
        });

        let expected_id = StandardId::new(NMEA_RX_ARBITRATION_ID).map(Id::Standard);
        let mut nmea_rx = NmeaCanReceiver::new();
        loop {
            match socket.read_frame().await {
                Ok(CanFrame::Data(frame)) => {
                    if Some(frame.id()) == expected_id {
                        last_rx_ms
                            .store(boot_instant.elapsed().as_millis() as u64, Ordering::Relaxed);
                        if let Err(e) = nmea_rx.handle_frame(
                            frame.data(),
                            state_mutex,
                            fault_registry,
                            foc_freq_hz,
                        ) {
                            debug!("SocketCAN RX decode error on {can_interface}: {e:?}");
                        }
                    }
                }
                Ok(CanFrame::Remote(_)) | Ok(CanFrame::Error(_)) => {}
                Err(e) => {
                    warn!("SocketCAN read error on {can_interface}: {e}");
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
            }
        }
    }
}

pub async fn run(
    can_interface: &str,
    foc_freq_hz: u32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        let force_multicast =
            can_interface.starts_with("mcast:") || can_interface.parse::<SocketAddrV4>().is_ok();
        if !force_multicast {
            match linux_socketcan::try_open(can_interface) {
                Ok(socket) => {
                    return linux_socketcan::run_with_socket(
                        can_interface,
                        socket,
                        foc_freq_hz,
                        state_mutex,
                        fault_registry,
                    )
                    .await;
                }
                Err(e) => {
                    info!(
                        "Kernel SocketCAN interface '{can_interface}' unavailable ({e}); falling back to UDP Multicast Virtual CAN bus"
                    );
                }
            }
        }
    }

    run_multicast(can_interface, foc_freq_hz, state_mutex, fault_registry).await
}
