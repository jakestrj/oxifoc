//! Virtual CAN-over-UDP server transporting `streaming-protocol` + `nmea` frames.
//!
//! Each UDP datagram carries a 10-byte CAN frame:
//! - Bytes `0..2`: 11-bit standard CAN arbitration ID in little-endian `u16`
//!   (`NMEA_RX_ARBITRATION_ID = 0x7E5` host -> device,
//!   `NMEA_TX_ARBITRATION_ID = 0x7ED` device -> host)
//! - Bytes `2..10`: 8-byte NMEA fast-packet CAN payload
//!
//! The server learns the active host's `SocketAddr` from incoming `0x7E5`
//! datagrams and directs `0x7ED` telemetry datagrams to that peer until
//! `LIVENESS_TIMEOUT_MS` elapses without incoming traffic.

use core::cell::RefCell;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use nmea::nmea_message::{MAX_NMEA_PACKET_SIZE, Message as NmeaMessage};
use oxifoc_core::foc::fault::{FaultRegistry, StandardFault};
use oxifoc_core::icd::{
    LIVENESS_TIMEOUT_MS, NMEA_INTER_FRAME_DELAY_MS, NMEA_RX_ARBITRATION_ID, NMEA_TX_ARBITRATION_ID,
};
use oxifoc_core::runtime::servers::{
    CommsError, MessageSender, NmeaCanReceiver, run_telemetry_publishers, send_messages,
};
use oxifoc_core::state::MotorControlState;
use tokio::net::UdpSocket;
use tracing::{debug, info};

use crate::tcp_server::CAN_WIRE_FRAME_LEN;

struct UdpCanSender {
    socket: Arc<UdpSocket>,
    peer: Arc<Mutex<Option<SocketAddr>>>,
}

impl MessageSender for UdpCanSender {
    async fn send_message_internal(&mut self, encoded_buf: &mut [u8]) -> Result<(), CommsError> {
        if encoded_buf.len() > MAX_NMEA_PACKET_SIZE {
            return Err(CommsError::NmeaError);
        }
        let peer_addr = self.peer.lock().ok().and_then(|g| *g);
        let Some(target) = peer_addr else {
            return Ok(());
        };
        let mut nmea_msg = NmeaMessage::from_payload(encoded_buf, 0);
        let id_bytes = NMEA_TX_ARBITRATION_ID.to_le_bytes();
        while let Some(frame) = nmea_msg.pop_frame() {
            let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
            wire[0..2].copy_from_slice(&id_bytes);
            wire[2..10].copy_from_slice(&frame.bytes);
            self.socket
                .send_to(&wire, target)
                .await
                .map_err(|_| CommsError::CanError)?;
            if nmea_msg.num_frames > 1 {
                tokio::time::sleep(Duration::from_millis(NMEA_INTER_FRAME_DELAY_MS)).await;
            }
        }
        Ok(())
    }
}

pub async fn run(
    port: u16,
    foc_freq_hz: u32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
) -> Result<()> {
    let bind_addr = format!("0.0.0.0:{port}");
    let sock = socket2::Socket::new(
        socket2::Domain::IPV4,
        socket2::Type::DGRAM,
        Some(socket2::Protocol::UDP),
    )?;
    sock.set_reuse_address(true)?;
    sock.set_nonblocking(true)?;
    sock.bind(&bind_addr.parse::<SocketAddr>()?.into())?;
    let socket = Arc::new(UdpSocket::from_std(sock.into())?);
    info!("CAN-over-UDP server listening on {bind_addr}");

    // Spawn persistent background telemetry publishers (slow, fast, fault watches).
    tokio::spawn(run_telemetry_publishers::<StandardFault, crate::TokioTimer>(
        state_mutex,
        fault_registry,
        foc_freq_hz,
    ));

    let peer: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(None));
    let boot_instant = Instant::now();
    let last_rx_ms = Arc::new(AtomicU64::new(0));

    // Outbound CAN TX task.
    tokio::spawn({
        let socket = socket.clone();
        let peer = peer.clone();
        async move {
            let mut sender = UdpCanSender { socket, peer };
            send_messages(&mut sender).await;
        }
    });

    // Liveness monitor task.
    tokio::spawn({
        let peer = peer.clone();
        let last_rx_ms = last_rx_ms.clone();
        async move {
            let mut interval = tokio::time::interval(Duration::from_millis(100));
            loop {
                interval.tick().await;
                let has_peer = peer.lock().ok().is_some_and(|g| g.is_some());
                if has_peer {
                    let elapsed = (boot_instant.elapsed().as_millis() as u64)
                        .saturating_sub(last_rx_ms.load(Ordering::Relaxed));
                    if elapsed > LIVENESS_TIMEOUT_MS {
                        if let Ok(mut g) = peer.lock() {
                            *g = None;
                        }
                        critical_section::with(|cs| {
                            state_mutex.borrow(cs).borrow_mut().set_link_inactive();
                        });
                    }
                }
            }
        }
    });

    // Inbound CAN RX loop.
    let mut nmea_rx = NmeaCanReceiver::new();
    let mut buf = [0u8; 64];
    loop {
        let (len, src_addr) = socket.recv_from(&mut buf).await?;
        if len != CAN_WIRE_FRAME_LEN {
            continue;
        }
        let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
        if arb_id == NMEA_RX_ARBITRATION_ID {
            let mut peer_changed = false;
            if let Ok(mut g) = peer.lock()
                && *g != Some(src_addr)
            {
                *g = Some(src_addr);
                peer_changed = true;
            }
            if peer_changed {
                nmea_rx.reset();
                info!("CAN-over-UDP learned peer: {src_addr}");
            }
            last_rx_ms.store(
                boot_instant.elapsed().as_millis() as u64,
                Ordering::Relaxed,
            );
            if let Err(e) = nmea_rx.handle_frame(
                &buf[2..10],
                state_mutex,
                fault_registry,
                foc_freq_hz,
            ) {
                debug!("CAN-over-UDP RX decode error from {src_addr}: {e:?}");
            }
        }
    }
}
