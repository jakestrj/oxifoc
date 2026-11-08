//! Virtual CAN-over-TCP server transporting `streaming-protocol` + `nmea` frames.
//!
//! Each classic 8-byte CAN frame is framed on the TCP stream as 10 bytes:
//! - Bytes `0..2`: 11-bit standard CAN arbitration ID in little-endian `u16`
//!   (`NMEA_RX_ARBITRATION_ID = 0x7E5` host -> device,
//!   `NMEA_TX_ARBITRATION_ID = 0x7ED` device -> host)
//! - Bytes `2..10`: 8-byte NMEA fast-packet CAN payload

use core::cell::RefCell;
use std::sync::Arc;
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
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::net::tcp::OwnedWriteHalf;
use tokio_util::sync::CancellationToken;
use tracing::{debug, info, warn};

/// Size of a framed CAN packet over TCP/UDP: 2-byte LE arbitration ID + 8-byte payload.
pub const CAN_WIRE_FRAME_LEN: usize = 10;

struct TcpCanSender {
    tx: OwnedWriteHalf,
}

impl MessageSender for TcpCanSender {
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
            self.tx
                .write_all(&wire)
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
    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;
    info!("CAN-over-TCP server listening on 0.0.0.0:{port}");

    // Spawn persistent background telemetry publishers (slow, fast, fault watches).
    tokio::spawn(run_telemetry_publishers::<StandardFault, crate::TokioTimer>(
        state_mutex,
        fault_registry,
        foc_freq_hz,
    ));

    let boot_instant = Instant::now();
    let mut prev_conn: Option<CancellationToken> = None;

    loop {
        let (socket, addr) = listener.accept().await?;
        let _ = socket.set_nodelay(true);
        info!("CAN-over-TCP client connected: {addr}");

        // Single active host connection: cancel the previous session immediately.
        if let Some(prev_token) = prev_conn.take() {
            prev_token.cancel();
            critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_link_inactive());
            tokio::task::yield_now().await;
        }

        let conn_token = CancellationToken::new();
        prev_conn = Some(conn_token.clone());

        let now_ms = boot_instant.elapsed().as_millis() as u64;
        let last_rx_ms = Arc::new(AtomicU64::new(now_ms));

        let (mut rx, tx) = socket.into_split();

        // Outbound CAN TX task (watches telemetry channels and writes 0x7ED frames).
        tokio::spawn({
            let token = conn_token.clone();
            async move {
                let mut sender = TcpCanSender { tx };
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = send_messages(&mut sender) => {}
                }
            }
        });

        // Liveness monitor task: marks link inactive if no RX frame arrives within LIVENESS_TIMEOUT_MS.
        tokio::spawn({
            let token = conn_token.clone();
            let last_rx_ms = last_rx_ms.clone();
            async move {
                let mut interval = tokio::time::interval(Duration::from_millis(100));
                loop {
                    tokio::select! {
                        _ = token.cancelled() => break,
                        _ = interval.tick() => {
                            let elapsed = (boot_instant.elapsed().as_millis() as u64)
                                .saturating_sub(last_rx_ms.load(Ordering::Relaxed));
                            if elapsed > LIVENESS_TIMEOUT_MS {
                                critical_section::with(|cs| {
                                    if !token.is_cancelled() {
                                        state_mutex.borrow(cs).borrow_mut().set_link_inactive();
                                    }
                                });
                            }
                        }
                    }
                }
            }
        });

        // Inbound CAN RX task (reads 10-byte CAN frames and dispatches 0x7E5 NMEA frames).
        tokio::spawn({
            let token = conn_token.clone();
            let last_rx_ms = last_rx_ms.clone();
            async move {
                let mut nmea_rx = NmeaCanReceiver::new();
                let mut buf = [0u8; CAN_WIRE_FRAME_LEN];
                loop {
                    let read_res = tokio::select! {
                        _ = token.cancelled() => break,
                        res = rx.read_exact(&mut buf) => res,
                    };
                    match read_res {
                        Ok(_) => {
                            let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
                            if arb_id == NMEA_RX_ARBITRATION_ID {
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
                                    debug!("CAN-over-TCP RX decode error: {e:?}");
                                }
                            }
                        }
                        Err(_) => {
                            warn!("CAN-over-TCP client disconnected: {addr}");
                            critical_section::with(|cs| {
                                if !token.is_cancelled() {
                                    state_mutex.borrow(cs).borrow_mut().set_link_inactive();
                                }
                            });
                            token.cancel();
                            break;
                        }
                    }
                }
            }
        });
    }
}
