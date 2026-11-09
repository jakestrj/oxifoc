//! Per-connection CAN (`streaming-protocol` + `nmea`) lifecycle and transport I/O
//! over `Can` (UDP Multicast Virtual CAN / Linux SocketCAN).

use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::Result;
use crossbeam_channel::Sender;
use femtopb::Message as _;
use oxifoc_core::proto::oxifoc::WireMessage;

use super::{LoggedWireMessage, transport};

/// Serialize an [`oxifoc_core::proto::oxifoc::WireMessage`] into raw protobuf bytes.
#[must_use]
pub fn encode_wire_message_proto(msg: &WireMessage<'_>) -> Vec<u8> {
    let len = msg.encoded_len();
    let mut buf = vec![0u8; len];
    let mut slice = buf.as_mut_slice();
    let _ = msg.encode(&mut slice);
    buf
}

pub(super) fn unix_timestamp_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

/// Link abstraction for sending `0x7E5` (`NMEA_RX_ARBITRATION_ID`)
/// `WireMessage` frames and receiving `0x7ED` (`NMEA_TX_ARBITRATION_ID`)
/// 8-byte NMEA CAN frames.
pub(super) struct CanFrameLink {
    endpoint: transport::can::CanEndpoint,
    wire_tx: Sender<LoggedWireMessage>,
}

impl CanFrameLink {
    pub(super) fn new(
        endpoint: transport::can::CanEndpoint,
        wire_tx: Sender<LoggedWireMessage>,
    ) -> Self {
        Self { endpoint, wire_tx }
    }

    pub(super) async fn send_wire_message(&mut self, msg: &WireMessage<'_>) -> Result<()> {
        self.endpoint.send_wire_message(msg).await?;
        let _ = self.wire_tx.try_send(LoggedWireMessage {
            timestamp_ns: unix_timestamp_ns(),
            data: encode_wire_message_proto(msg),
        });
        Ok(())
    }

    pub(super) async fn recv_frame(&mut self) -> Result<(u16, [u8; 8])> {
        self.endpoint.recv_frame().await
    }
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn protocol_is_compatible(device_version: u16) -> bool {
    device_version == oxifoc_core::types::ICD_PROTO_VERSION
}
