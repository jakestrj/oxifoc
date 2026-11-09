//! Per-connection CAN (`streaming-protocol` + `nmea`) lifecycle and transport I/O
//! shared across `Can` (UDP Multicast Virtual CAN / Linux SocketCAN), `Tcp`,
//! `Udp`, `Serial`, and `Rtt`.

use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use oxifoc_core::icd::{
    CAN_WIRE_FRAME_LEN, NMEA_INTER_FRAME_DELAY_MS, NMEA_RX_ARBITRATION_ID,
};
use oxifoc_core::proto::oxifoc::WireMessage;
use oxifoc_core::runtime::servers::encode_wire_message_frames;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UdpSocket;

use super::transport;

/// Unified link abstraction for sending `0x7E5` (`NMEA_RX_ARBITRATION_ID`)
/// `WireMessage` frames and receiving `0x7ED` (`NMEA_TX_ARBITRATION_ID`)
/// 8-byte NMEA CAN frames.
pub(super) enum CanFrameLink {
    Can(transport::can::CanEndpoint),
    Stream(transport::CobsStreamTransport),
    Udp(Arc<UdpSocket>),
}

impl CanFrameLink {
    pub(super) async fn send_wire_message(&mut self, msg: &WireMessage<'_>) -> Result<()> {
        match self {
            Self::Can(ep) => ep.send_wire_message(msg).await,
            Self::Stream(stream) => {
                let mut nmea_msg = encode_wire_message_frames(msg)
                    .map_err(|e| anyhow::anyhow!("encode wire message failed: {e:?}"))?;
                let id_bytes = NMEA_RX_ARBITRATION_ID.to_le_bytes();
                while let Some(frame) = nmea_msg.pop_frame() {
                    let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
                    wire[0..2].copy_from_slice(&id_bytes);
                    wire[2..10].copy_from_slice(&frame.bytes);
                    stream
                        .writer
                        .write_all(&wire)
                        .await
                        .context("write CAN frame to stream")?;
                    if nmea_msg.num_frames > 1 {
                        tokio::time::sleep(Duration::from_millis(NMEA_INTER_FRAME_DELAY_MS)).await;
                    }
                }
                Ok(())
            }
            Self::Udp(socket) => {
                let mut nmea_msg = encode_wire_message_frames(msg)
                    .map_err(|e| anyhow::anyhow!("encode wire message failed: {e:?}"))?;
                let id_bytes = NMEA_RX_ARBITRATION_ID.to_le_bytes();
                while let Some(frame) = nmea_msg.pop_frame() {
                    let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
                    wire[0..2].copy_from_slice(&id_bytes);
                    wire[2..10].copy_from_slice(&frame.bytes);
                    socket
                        .send(&wire)
                        .await
                        .context("send CAN frame over UDP")?;
                    if nmea_msg.num_frames > 1 {
                        tokio::time::sleep(Duration::from_millis(NMEA_INTER_FRAME_DELAY_MS)).await;
                    }
                }
                Ok(())
            }
        }
    }

    pub(super) async fn recv_frame(&mut self) -> Result<(u16, [u8; 8])> {
        match self {
            Self::Can(ep) => ep.recv_frame().await,
            Self::Stream(stream) => {
                let mut buf = [0u8; CAN_WIRE_FRAME_LEN];
                stream
                    .reader
                    .read_exact(&mut buf)
                    .await
                    .context("read CAN frame from stream")?;
                let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
                let mut data = [0u8; 8];
                data.copy_from_slice(&buf[2..10]);
                Ok((arb_id, data))
            }
            Self::Udp(socket) => {
                let mut buf = [0u8; 64];
                loop {
                    let len = socket
                        .recv(&mut buf)
                        .await
                        .context("recv CAN frame over UDP")?;
                    if len == CAN_WIRE_FRAME_LEN {
                        let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
                        let mut data = [0u8; 8];
                        data.copy_from_slice(&buf[2..10]);
                        return Ok((arb_id, data));
                    }
                }
            }
        }
    }

    pub(super) fn take_defmt_reader(
        &mut self,
    ) -> Option<Box<dyn tokio::io::AsyncRead + Send + Unpin>> {
        match self {
            Self::Stream(s) => s.defmt_reader.take(),
            Self::Can(_) | Self::Udp(_) => None,
        }
    }
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn protocol_is_compatible(device_version: u16) -> bool {
    device_version == oxifoc_core::types::ICD_PROTO_VERSION
}
