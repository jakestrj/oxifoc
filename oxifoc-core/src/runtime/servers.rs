//! Streaming-protocol + NMEA over CAN communication handlers and servers.
//!
//! Modeled after `hv-bms-firmware-rs/src/comms/message.rs` and `can_task.rs`:
//! - Outgoing telemetry messages (`SlowTelemetryMessage`, `FastTelemetryMessage`,
//!   `FaultStateMessage`) are watched via `SLOW_TELEMETRY_WATCH`,
//!   `FAST_TELEMETRY_WATCH`, and `FAULT_STATE_WATCH`, encoded with
//!   `streaming_protocol::encoder::Encoder::encode_proto`, and fragmented into
//!   8-byte CAN frames using `nmea::nmea_message::Message::from_payload`.
//! - Incoming host commands (`MotorCommandRequest`, `FaultClearRequest`,
//!   `TelemetryConfigRequest`, `PhaseSourceRequest`) are reassembled from
//!   8-byte NMEA CAN frames, decoded with
//!   `streaming_protocol::decoder::Decoder`, and dispatched to `STATE`,
//!   `CMD_CHANNEL`, `FAULT_REGISTRY`, and `FAST_TELEM_PERIOD`.

use core::cell::RefCell;
use core::sync::atomic::Ordering;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_futures::join::join3;
use embassy_futures::select::{Either3, select3};
use femtopb::{EnumValue, UnknownFields};
use nmea::nmea_message::{MAX_NMEA_PACKET_SIZE, Message as NmeaMessage};
use streaming_protocol::decoder::Decoder;
use streaming_protocol::encoder::Encoder;

use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
use crate::foc::phase::PhaseSource;
use crate::icd::{DECODER_BUF_LEN, ENCODER_BUF_LEN, MSG_ID, MotorRequest};
use crate::proto::oxifoc::{WireMessage, wire_message::Payload};
use crate::runtime::streaming::{
    FAST_TELEM_PERIOD, FAST_TELEMETRY_WATCH, FAULT_STATE_WATCH, SLOW_TELEMETRY_WATCH, cmd_stats,
    fast_telem_stats, fast_telemetry_stream, fault_topic_stream, slow_telemetry_stream,
};
use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState};
use crate::timer::Timer;

/// Default interval in milliseconds for slow telemetry publication (10 Hz).
pub const DEFAULT_SLOW_TELEM_INTERVAL_MS: u64 = 100;

/// Communication errors for `streaming-protocol` + `nmea` CAN transport.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommsError {
    /// NMEA fast-packet fragmentation or reassembly error.
    NmeaError,
    /// `streaming-protocol` COBS/protobuf encoding error.
    StreamingProtocolEncodeError,
    /// `streaming-protocol` COBS/CRC/protobuf decoding error.
    StreamingProtocolDecodeError,
    /// Underlying CAN bus or transport transmit/receive error.
    CanError,
    /// Unknown or malformed request payload.
    InvalidRequest,
}

/// Trait implemented by CAN TX drivers (or virtual transports) to send
/// `streaming-protocol` encoded messages.
#[allow(async_fn_in_trait)]
pub trait MessageSender {
    /// Encode a [`WireMessage`] with `streaming-protocol` and transmit it.
    async fn send_message(&mut self, msg: WireMessage<'_>) -> Result<(), CommsError> {
        let mut encoded_buf = [0u8; ENCODER_BUF_LEN];
        let wire_len = Encoder::encode_proto(&msg, MSG_ID, &mut encoded_buf)
            .map_err(|_| CommsError::StreamingProtocolEncodeError)?;
        let Some(slice) = encoded_buf.get_mut(..wire_len) else {
            return Err(CommsError::StreamingProtocolEncodeError);
        };
        self.send_message_internal(slice).await
    }

    /// Fragment and transmit the already-encoded `streaming-protocol` buffer.
    async fn send_message_internal(&mut self, encoded_buf: &mut [u8]) -> Result<(), CommsError>;
}

/// Encode a [`WireMessage`] with `streaming-protocol` and fragment it into an
/// [`NmeaMessage`] (sequence of 8-byte CAN frames).
pub fn encode_wire_message_frames(msg: &WireMessage<'_>) -> Result<NmeaMessage, CommsError> {
    let mut encoded_buf = [0u8; ENCODER_BUF_LEN];
    let wire_len = Encoder::encode_proto(msg, MSG_ID, &mut encoded_buf)
        .map_err(|_| CommsError::StreamingProtocolEncodeError)?;
    if wire_len > MAX_NMEA_PACKET_SIZE {
        return Err(CommsError::NmeaError);
    }
    let Some(slice) = encoded_buf.get(..wire_len) else {
        return Err(CommsError::StreamingProtocolEncodeError);
    };
    Ok(NmeaMessage::from_payload(slice, 0))
}

/// Continuously watch `SLOW_TELEMETRY_WATCH`, `FAST_TELEMETRY_WATCH`, and
/// `FAULT_STATE_WATCH` and transmit updates via `sender`.
pub async fn send_messages<T: MessageSender>(sender: &mut T) {
    let Some(mut slow_rx) = SLOW_TELEMETRY_WATCH.receiver() else {
        return;
    };
    let Some(mut fast_rx) = FAST_TELEMETRY_WATCH.receiver() else {
        return;
    };
    let Some(mut fault_rx) = FAULT_STATE_WATCH.receiver() else {
        return;
    };

    loop {
        let msg: WireMessage<'_> =
            match select3(slow_rx.changed(), fast_rx.changed(), fault_rx.changed()).await {
                Either3::First(slow) => WireMessage {
                    payload: Some(Payload::SlowTelemetry(slow.into())),
                    unknown_fields: UnknownFields::empty(),
                },
                Either3::Second(fast) => WireMessage {
                    payload: Some(Payload::FastTelemetry(fast.into())),
                    unknown_fields: UnknownFields::empty(),
                },
                Either3::Third(fault) => WireMessage {
                    payload: Some(Payload::FaultState(fault.into())),
                    unknown_fields: UnknownFields::empty(),
                },
            };

        if sender.send_message(msg).await.is_ok() {
            fast_telem_stats::BCAST_OK.fetch_add(1, Ordering::Relaxed);
        } else {
            fast_telem_stats::BCAST_FAILS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Process a decoded [`WireMessage`] request from the host and apply it to
/// `state_mutex`, `CMD_CHANNEL`, `fault_registry`, and `FAST_TELEM_PERIOD`.
pub fn process_wire_message<F: PlatformFault>(
    msg: &WireMessage<'_>,
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &FaultRegistry<F>,
    foc_freq_hz: u32,
) -> Result<(), CommsError> {
    let Some(payload) = msg.payload.as_ref() else {
        return Err(CommsError::InvalidRequest);
    };

    // Any valid incoming request marks the communication link active.
    critical_section::with(|cs| {
        state_mutex.borrow(cs).borrow_mut().set_link_active();
    });

    match payload {
        Payload::MotorCommand(cmd_req) => {
            let Some(request) = MotorRequest::from_proto(cmd_req) else {
                return Err(CommsError::InvalidRequest);
            };
            cmd_stats::MOTOR_REQS.fetch_add(1, Ordering::Relaxed);
            let _ = CMD_CHANNEL.try_send(DriverCommand::Motor(request));
            Ok(())
        }
        Payload::FaultClear(clear_req) => {
            if clear_req.clear_all {
                if clear_req.expected_generation == 0 {
                    fault_registry.clear_all();
                } else {
                    let _ = fault_registry.clear_if_generation(clear_req.expected_generation, None);
                }
            } else if let EnumValue::Known(proto_cat) = clear_req.category {
                let category: FaultCategory = proto_cat.into();
                if clear_req.expected_generation == 0 {
                    fault_registry.clear(category);
                } else {
                    let _ = fault_registry
                        .clear_if_generation(clear_req.expected_generation, Some(category));
                }
            } else {
                return Err(CommsError::InvalidRequest);
            }
            Ok(())
        }
        Payload::TelemetryConfig(cfg_req) => {
            let period = if cfg_req.fast_hz > 0 {
                (foc_freq_hz / cfg_req.fast_hz.max(1)).max(1)
            } else {
                0
            };
            FAST_TELEM_PERIOD.store(period, Ordering::Relaxed);
            Ok(())
        }
        Payload::PhaseSourceRequest(ps_req) => {
            let Some(source) = PhaseSource::from_proto_request(ps_req) else {
                return Err(CommsError::InvalidRequest);
            };
            let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(source));
            Ok(())
        }
        Payload::SlowTelemetry(_) | Payload::FastTelemetry(_) | Payload::FaultState(_) | _ => {
            Err(CommsError::InvalidRequest)
        }
    }
}

/// Reassembles incoming 8-byte NMEA CAN frames and decodes `streaming-protocol`
/// [`WireMessage`] payloads.
pub struct NmeaCanReceiver {
    message: NmeaMessage,
}

impl NmeaCanReceiver {
    /// Create a new empty NMEA fast-packet receiver.
    pub fn new() -> Self {
        Self {
            message: NmeaMessage::new(),
        }
    }

    /// Reset the receiver state.
    pub fn reset(&mut self) {
        self.message.clear();
    }

    /// Push a single 8-byte CAN frame. When all frames of an NMEA message have
    /// arrived, returns the reassembled payload buffer and its byte length.
    pub fn push_frame(
        &mut self,
        frame_data: &[u8],
    ) -> Result<Option<([u8; MAX_NMEA_PACKET_SIZE], usize)>, CommsError> {
        let Ok(frame_arr): Result<&[u8; 8], _> = frame_data.try_into() else {
            self.reset();
            return Err(CommsError::NmeaError);
        };
        let Ok(finished) = self.message.add_frame(frame_arr) else {
            self.reset();
            return Err(CommsError::NmeaError);
        };
        let single_frame_complete = self.message.num_frames == 1 && self.message.data_len > 0;
        if !finished && !single_frame_complete {
            return Ok(None);
        }

        let mut payload_buf = [0xFFu8; MAX_NMEA_PACKET_SIZE];
        let payload_len = self.message.get_payload(&mut payload_buf);
        self.reset();

        if payload_len > MAX_NMEA_PACKET_SIZE {
            return Err(CommsError::NmeaError);
        }

        Ok(Some((payload_buf, payload_len)))
    }

    /// Push a single 8-byte CAN frame and, once a full NMEA message is
    /// assembled, decode the [`WireMessage`] and invoke `f(&proto)`.
    pub fn push_frame_with<R>(
        &mut self,
        frame_data: &[u8],
        f: impl FnOnce(&WireMessage<'_>) -> Result<R, CommsError>,
    ) -> Result<Option<R>, CommsError> {
        let Some((payload_buf, payload_len)) = self.push_frame(frame_data)? else {
            return Ok(None);
        };
        let Some(payload_slice) = payload_buf.get(..payload_len) else {
            return Err(CommsError::NmeaError);
        };
        let mut decoder_buf = [0u8; DECODER_BUF_LEN];
        let pkt = Decoder::decode_buf(payload_slice, &mut decoder_buf)
            .map_err(|_| CommsError::StreamingProtocolDecodeError)?;
        let proto: WireMessage<'_> =
            Decoder::decode_proto(&pkt).map_err(|_| CommsError::StreamingProtocolDecodeError)?;
        f(&proto).map(Some)
    }

    /// Feed an 8-byte CAN frame and, once a full NMEA message is assembled,
    /// decode and dispatch it via [`process_wire_message`].
    pub fn handle_frame<F: PlatformFault>(
        &mut self,
        frame_data: &[u8],
        state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
        fault_registry: &FaultRegistry<F>,
        foc_freq_hz: u32,
    ) -> Result<bool, CommsError> {
        let res = self.push_frame_with(frame_data, |proto| {
            process_wire_message(proto, state_mutex, fault_registry, foc_freq_hz)
        })?;
        Ok(res.is_some())
    }
}

impl Default for NmeaCanReceiver {
    fn default() -> Self {
        Self::new()
    }
}

/// Run the background telemetry publisher streams (`slow_telemetry_stream`,
/// `fast_telemetry_stream`, and `fault_topic_stream`) concurrently.
pub async fn run_telemetry_publishers<F: PlatformFault, T: Timer>(
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    foc_freq_hz: u32,
) {
    join3(
        slow_telemetry_stream::<F, T>(state_mutex, fault_registry, DEFAULT_SLOW_TELEM_INTERVAL_MS),
        fast_telemetry_stream::<T>(foc_freq_hz),
        fault_topic_stream(fault_registry),
    )
    .await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::fault::{FaultSeverity, StandardFault};
    use crate::icd::FaultStateSnapshot;
    use crate::types::{ControlMode, FastTelemetry, MotorCommand, MotorState, SlowTelemetry};

    #[test]
    fn nmea_streaming_protocol_roundtrip_telemetry_and_commands() {
        let slow = SlowTelemetry {
            vbus_mv: 48_000,
            fet_temp_c_x10: 325,
            motor_temp_c_x10: 410,
            board_temp_c_x10: 300,
            motor_state: MotorState::Running,
            control_mode: ControlMode::CurrentControl {
                iq_target: 12.5,
                id_target: -1.0,
            },
            fault_count: 1,
            phase_source: PhaseSource::Observer,
            seq: 42,
            derate_drive_pct: 95,
            derate_brake_pct: 100,
            fault_generation: 3,
            derate_regen_pct: 80,
        };
        let wire_msg = WireMessage {
            payload: Some(Payload::SlowTelemetry(slow.into())),
            unknown_fields: UnknownFields::empty(),
        };
        let mut nmea_tx = encode_wire_message_frames(&wire_msg).unwrap();
        let mut rx = NmeaCanReceiver::new();
        let mut decoded_slow = None;
        while let Some(frame) = nmea_tx.pop_frame() {
            if let Some(res) = rx
                .push_frame_with(&frame.bytes, |proto| match proto.payload.as_ref() {
                    Some(Payload::SlowTelemetry(m)) => Ok(SlowTelemetry::from(m)),
                    _ => Err(CommsError::InvalidRequest),
                })
                .unwrap()
            {
                decoded_slow = Some(res);
            }
        }
        let decoded_slow = decoded_slow.unwrap();
        assert_eq!(decoded_slow.vbus_mv, slow.vbus_mv);
        assert_eq!(decoded_slow.fet_temp_c_x10, slow.fet_temp_c_x10);
        assert_eq!(decoded_slow.motor_state, slow.motor_state);
        assert_eq!(decoded_slow.control_mode, slow.control_mode);
        assert_eq!(decoded_slow.phase_source, slow.phase_source);
        assert_eq!(decoded_slow.seq, slow.seq);

        // FastTelemetry roundtrip
        let fast = FastTelemetry {
            ia: 2048,
            ib: 2050,
            ic: 2046,
            vbus: 24_000,
            angle: 16384,
            vd: -120,
            vq: 1400,
            rpm: 1500,
            seq: 999,
        };
        let fast_wire = WireMessage {
            payload: Some(Payload::FastTelemetry(fast.into())),
            unknown_fields: UnknownFields::empty(),
        };
        let mut nmea_tx = encode_wire_message_frames(&fast_wire).unwrap();
        let mut decoded_fast = None;
        while let Some(frame) = nmea_tx.pop_frame() {
            if let Some(res) = rx
                .push_frame_with(&frame.bytes, |proto| match proto.payload.as_ref() {
                    Some(Payload::FastTelemetry(m)) => Ok(FastTelemetry::from(m)),
                    _ => Err(CommsError::InvalidRequest),
                })
                .unwrap()
            {
                decoded_fast = Some(res);
            }
        }
        assert_eq!(decoded_fast.unwrap(), fast);

        // FaultStateSnapshot roundtrip
        let fault_snap = FaultStateSnapshot {
            generation: 7,
            active_categories_mask: 1 << (FaultCategory::OverVoltage as u32),
            total: 1,
            primary_category: FaultCategory::OverVoltage,
            max_severity: FaultSeverity::Kill,
        };
        let fault_wire = WireMessage {
            payload: Some(Payload::FaultState(fault_snap.into())),
            unknown_fields: UnknownFields::empty(),
        };
        let mut nmea_tx = encode_wire_message_frames(&fault_wire).unwrap();
        let mut decoded_fault = None;
        while let Some(frame) = nmea_tx.pop_frame() {
            if let Some(res) = rx
                .push_frame_with(&frame.bytes, |proto| match proto.payload.as_ref() {
                    Some(Payload::FaultState(m)) => Ok(FaultStateSnapshot::from(m)),
                    _ => Err(CommsError::InvalidRequest),
                })
                .unwrap()
            {
                decoded_fault = Some(res);
            }
        }
        assert_eq!(decoded_fault.unwrap(), fault_snap);
    }

    #[test]
    fn handle_frame_dispatches_motor_and_fault_commands() {
        let state = CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let faults: FaultRegistry<StandardFault> = FaultRegistry::new();
        faults.set(StandardFault::OverVoltage);
        assert!(faults.any());

        // Clear fault via NMEA + streaming-protocol frames
        let clear_wire = WireMessage {
            payload: Some(Payload::FaultClear(
                crate::proto::oxifoc::FaultClearRequest {
                    expected_generation: 0,
                    clear_all: true,
                    category: EnumValue::Known(crate::proto::oxifoc::FaultCategory::None),
                    unknown_fields: UnknownFields::empty(),
                },
            )),
            unknown_fields: UnknownFields::empty(),
        };
        let mut nmea_tx = encode_wire_message_frames(&clear_wire).unwrap();
        let mut rx = NmeaCanReceiver::new();
        let mut handled = false;
        while let Some(frame) = nmea_tx.pop_frame() {
            if rx
                .handle_frame(&frame.bytes, &state, &faults, 20_000)
                .unwrap()
            {
                handled = true;
            }
        }
        assert!(handled);
        assert!(!faults.any());

        // Send MotorCommand via NMEA + streaming-protocol frames
        let motor_req = MotorRequest {
            source_session: 0x1234,
            seq: 1,
            command: MotorCommand::SetMode(ControlMode::VelocityControl { target_vel: 150.0 }),
        };
        let cmd_wire = WireMessage {
            payload: Some(Payload::MotorCommand(motor_req.into())),
            unknown_fields: UnknownFields::empty(),
        };
        let mut nmea_tx = encode_wire_message_frames(&cmd_wire).unwrap();
        let mut handled_cmd = false;
        while let Some(frame) = nmea_tx.pop_frame() {
            if rx
                .handle_frame(&frame.bytes, &state, &faults, 20_000)
                .unwrap()
            {
                handled_cmd = true;
            }
        }
        assert!(handled_cmd);
        let active = critical_section::with(|cs| state.borrow(cs).borrow().link_active);
        assert!(active);
    }
}
