//! Per-connection CAN (`streaming-protocol` + `nmea`) lifecycle and transport I/O
//! over `Can` (UDP Multicast Virtual CAN / Linux SocketCAN).

use anyhow::Result;
use oxifoc_core::proto::oxifoc::WireMessage;

use super::transport;

/// Link abstraction for sending `0x7E5` (`NMEA_RX_ARBITRATION_ID`)
/// `WireMessage` frames and receiving `0x7ED` (`NMEA_TX_ARBITRATION_ID`)
/// 8-byte NMEA CAN frames.
pub(super) struct CanFrameLink {
    endpoint: transport::can::CanEndpoint,
}

impl CanFrameLink {
    pub(super) fn new(endpoint: transport::can::CanEndpoint) -> Self {
        Self { endpoint }
    }

    pub(super) async fn send_wire_message(&mut self, msg: &WireMessage<'_>) -> Result<()> {
        self.endpoint.send_wire_message(msg).await
    }

    pub(super) async fn recv_frame(&mut self) -> Result<(u16, [u8; 8])> {
        self.endpoint.recv_frame().await
    }
}

#[cfg_attr(not(test), allow(dead_code))]
pub(super) fn protocol_is_compatible(device_version: u16) -> bool {
    device_version == oxifoc_core::types::ICD_PROTO_VERSION
}
