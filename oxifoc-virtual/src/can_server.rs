//! SocketCAN (`can0` / `vcan0`) server transporting `streaming-protocol` + `nmea` frames.
//!
//! Uses standard 11-bit CAN data frames:
//! - `NMEA_RX_ARBITRATION_ID = 0x7E5` (host -> device)
//! - `NMEA_TX_ARBITRATION_ID = 0x7ED` (device -> host)

use core::cell::RefCell;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
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
use socketcan::tokio::CanSocket;
use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};
use tracing::{debug, info, warn};

struct SocketCanSender {
    socket: Arc<CanSocket>,
}

impl MessageSender for SocketCanSender {
    async fn send_message_internal(&mut self, encoded_buf: &mut [u8]) -> Result<(), CommsError> {
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

pub async fn run(
    can_interface: &str,
    foc_freq_hz: u32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
) -> Result<()> {
    let socket = Arc::new(
        CanSocket::open(can_interface)
            .with_context(|| format!("failed to open SocketCAN interface '{can_interface}'"))?,
    );
    info!("SocketCAN server listening on {can_interface} (RX=0x7E5, TX=0x7ED)");

    // Spawn persistent background telemetry publishers (slow, fast, fault watches).
    tokio::spawn(run_telemetry_publishers::<StandardFault, crate::TokioTimer>(
        state_mutex,
        fault_registry,
        foc_freq_hz,
    ));

    let boot_instant = Instant::now();
    let last_rx_ms = Arc::new(AtomicU64::new(0));

    // Outbound CAN TX task (watches telemetry channels and writes 0x7ED frames).
    tokio::spawn({
        let socket = socket.clone();
        async move {
            let mut sender = SocketCanSender { socket };
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

    // Inbound CAN RX loop (reads 0x7E5 NMEA frames).
    let expected_id = StandardId::new(NMEA_RX_ARBITRATION_ID).map(Id::Standard);
    let mut nmea_rx = NmeaCanReceiver::new();
    loop {
        match socket.read_frame().await {
            Ok(CanFrame::Data(frame)) => {
                if Some(frame.id()) == expected_id {
                    last_rx_ms.store(
                        boot_instant.elapsed().as_millis() as u64,
                        Ordering::Relaxed,
                    );
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
