//! Interface Control Document (ICD) for Motor Control Protocol
//!
//! Defines the wire protocol constants and conversions between internal
//! `oxifoc-core` types and `femtopb` protobuf definitions encoded via
//! `streaming-protocol` and fragmented with `nmea` over CAN.

use femtopb::{EnumValue, UnknownFields};

use crate::foc::fault::FaultSeverity;
use crate::foc::phase::PhaseSource;
pub use crate::types::*;

/// Message ID used with `streaming_protocol::encoder::Encoder::encode_proto`.
pub const MSG_ID: u8 = 1;

/// Buffer capacity for `streaming-protocol` encoding (COBS + header + proto + CRC).
pub const ENCODER_BUF_LEN: usize = 256;

/// Buffer capacity for `streaming-protocol` decoding.
pub const DECODER_BUF_LEN: usize = 256;

/// Default CAN bus bitrate in bits per second (500 kbps).
pub const CAN_BITRATE: u32 = 500_000;

/// Standard 11-bit CAN arbitration ID for incoming host -> device NMEA frames.
pub const NMEA_RX_ARBITRATION_ID: u16 = 0x7E5;

/// Standard 11-bit CAN arbitration ID for outgoing device -> host NMEA frames.
pub const NMEA_TX_ARBITRATION_ID: u16 = 0x7ED;

/// Delay in milliseconds between consecutive 8-byte NMEA CAN frames.
pub const NMEA_INTER_FRAME_DELAY_MS: u64 = 2;

/// Liveness timeout in milliseconds for communication link monitoring.
pub const LIVENESS_TIMEOUT_MS: u64 = 3000;

/// Size of a framed classic 8-byte CAN packet over TCP/UDP/Multicast:
/// 2-byte little-endian 11-bit arbitration ID + 8-byte NMEA payload.
pub const CAN_WIRE_FRAME_LEN: usize = 10;

/// IPv4 Local Scope (RFC 2365) multicast group used for the cross-platform
/// UDP Multicast Virtual CAN (`vcan`) bus (`239.255.79.88`).
pub const VCAN_MULTICAST_IP: [u8; 4] = [239, 255, 79, 88];

/// Base UDP port for `vcan0` / `can0` on the UDP Multicast Virtual CAN bus.
pub const VCAN_MULTICAST_BASE_PORT: u16 = 29536;

/// Deterministically map a CAN interface name (e.g. `"vcan0"`, `"can1"`, or a
/// custom bus name) to a UDP port for the UDP Multicast Virtual CAN bus.
#[must_use]
pub fn vcan_multicast_port(can_interface: &str) -> u16 {
    let name = can_interface.strip_prefix("mcast:").unwrap_or(can_interface);
    if let Some(num_str) = name
        .strip_prefix("vcan")
        .or_else(|| name.strip_prefix("can"))
        && let Ok(idx) = num_str.parse::<u16>()
        && idx < 16
    {
        return VCAN_MULTICAST_BASE_PORT + idx;
    }
    // FNV-1a 32-bit hash mapped into [VCAN_MULTICAST_BASE_PORT + 16, VCAN_MULTICAST_BASE_PORT + 1000)
    let mut hash: u32 = 0x811c_9dc5;
    for &b in name.as_bytes() {
        hash ^= u32::from(b);
        hash = hash.wrapping_mul(0x0100_0193);
    }
    VCAN_MULTICAST_BASE_PORT + 16 + (hash % 984) as u16
}

/// Compact, `Copy` snapshot of active fault state for `Watch`-based streaming.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultStateSnapshot {
    /// Monotonic fault registry generation.
    pub generation: u32,
    /// Bitmask of active `FaultCategory` discriminants (`1 << (cat as u32)`).
    pub active_categories_mask: u32,
    /// Total number of active faults in the registry.
    pub total: u8,
    /// Primary (first) active fault category, or `FaultCategory::None`.
    pub primary_category: FaultCategory,
    /// Highest severity among active faults (`Warning` when no faults are active).
    pub max_severity: FaultSeverity,
}

impl FaultStateSnapshot {
    /// Build a compact `FaultStateSnapshot` from a `FaultSnapshot`.
    pub fn from_snapshot(snap: &FaultSnapshot) -> Self {
        let mut mask = 0u32;
        let mut primary = FaultCategory::None;
        let mut max_sev = FaultSeverity::Warning;
        for (idx, f) in snap.faults.iter().enumerate() {
            if idx == 0 {
                primary = f.category;
            }
            mask |= 1u32 << (f.category as u32);
            if f.severity > max_sev {
                max_sev = f.severity;
            }
        }
        Self {
            generation: snap.generation,
            active_categories_mask: mask,
            total: snap.total,
            primary_category: primary,
            max_severity: max_sev,
        }
    }

    /// Reconstruct a [`FaultSnapshot`] from the compact bitmask representation.
    #[must_use]
    pub fn to_fault_snapshot(self) -> FaultSnapshot {
        const ALL_CATEGORIES: [(FaultCategory, &str); 10] = [
            (FaultCategory::OverCurrent, "Over-current"),
            (FaultCategory::OverVoltage, "Over-voltage"),
            (FaultCategory::UnderVoltage, "Under-voltage"),
            (FaultCategory::OverTemp, "Over-temperature"),
            (FaultCategory::DriverFault, "Driver fault"),
            (FaultCategory::HallError, "Hall sensor error"),
            (FaultCategory::Stall, "Motor stall"),
            (FaultCategory::CalibrationFault, "Calibration fault"),
            (FaultCategory::CommTimeout, "Communication timeout"),
            (FaultCategory::Derating, "Thermal/voltage derating"),
        ];
        let mut faults = heapless::Vec::new();
        for (cat, desc) in ALL_CATEGORIES {
            if (self.active_categories_mask & (1u32 << (cat as u32))) != 0 {
                let _ = faults.push(FaultInfo::new(cat, desc));
            }
        }
        FaultSnapshot {
            generation: self.generation,
            faults,
            total: self.total,
        }
    }
}

// ============================================================================
// Protobuf Conversions
// ============================================================================

impl From<MotorState> for crate::proto::oxifoc::MotorState {
    fn from(state: MotorState) -> Self {
        match state {
            MotorState::Stopped => Self::Stopped,
            MotorState::Running => Self::Running,
            MotorState::Error => Self::Error,
        }
    }
}

impl From<crate::proto::oxifoc::MotorState> for MotorState {
    fn from(state: crate::proto::oxifoc::MotorState) -> Self {
        match state {
            crate::proto::oxifoc::MotorState::Stopped => Self::Stopped,
            crate::proto::oxifoc::MotorState::Running => Self::Running,
            crate::proto::oxifoc::MotorState::Error => Self::Error,
        }
    }
}

impl From<FaultCategory> for crate::proto::oxifoc::FaultCategory {
    fn from(cat: FaultCategory) -> Self {
        match cat {
            FaultCategory::None => Self::None,
            FaultCategory::OverCurrent => Self::OverCurrent,
            FaultCategory::OverVoltage => Self::OverVoltage,
            FaultCategory::UnderVoltage => Self::UnderVoltage,
            FaultCategory::OverTemp => Self::OverTemp,
            FaultCategory::DriverFault => Self::DriverFault,
            FaultCategory::HallError => Self::HallError,
            FaultCategory::Stall => Self::Stall,
            FaultCategory::CalibrationFault => Self::CalibrationFault,
            FaultCategory::CommTimeout => Self::CommTimeout,
            FaultCategory::Derating => Self::Derating,
        }
    }
}

impl From<crate::proto::oxifoc::FaultCategory> for FaultCategory {
    fn from(cat: crate::proto::oxifoc::FaultCategory) -> Self {
        match cat {
            crate::proto::oxifoc::FaultCategory::None => Self::None,
            crate::proto::oxifoc::FaultCategory::OverCurrent => Self::OverCurrent,
            crate::proto::oxifoc::FaultCategory::OverVoltage => Self::OverVoltage,
            crate::proto::oxifoc::FaultCategory::UnderVoltage => Self::UnderVoltage,
            crate::proto::oxifoc::FaultCategory::OverTemp => Self::OverTemp,
            crate::proto::oxifoc::FaultCategory::DriverFault => Self::DriverFault,
            crate::proto::oxifoc::FaultCategory::HallError => Self::HallError,
            crate::proto::oxifoc::FaultCategory::Stall => Self::Stall,
            crate::proto::oxifoc::FaultCategory::CalibrationFault => Self::CalibrationFault,
            crate::proto::oxifoc::FaultCategory::CommTimeout => Self::CommTimeout,
            crate::proto::oxifoc::FaultCategory::Derating => Self::Derating,
        }
    }
}

impl From<FaultSeverity> for crate::proto::oxifoc::FaultSeverity {
    fn from(sev: FaultSeverity) -> Self {
        match sev {
            FaultSeverity::Warning => Self::Warning,
            FaultSeverity::GracefulStop => Self::GracefulStop,
            FaultSeverity::Kill => Self::Kill,
        }
    }
}

impl From<crate::proto::oxifoc::FaultSeverity> for FaultSeverity {
    fn from(sev: crate::proto::oxifoc::FaultSeverity) -> Self {
        match sev {
            crate::proto::oxifoc::FaultSeverity::Warning => Self::Warning,
            crate::proto::oxifoc::FaultSeverity::GracefulStop => Self::GracefulStop,
            crate::proto::oxifoc::FaultSeverity::Kill => Self::Kill,
        }
    }
}

impl From<PhaseSource> for crate::proto::oxifoc::PhaseSourceKind {
    fn from(src: PhaseSource) -> Self {
        match src {
            PhaseSource::Hall => Self::PhaseSourceHall,
            PhaseSource::Encoder => Self::PhaseSourceEncoder,
            PhaseSource::Observer => Self::PhaseSourceObserver,
            PhaseSource::Hfi => Self::PhaseSourceHfi,
            PhaseSource::HallToObserver { .. } => Self::PhaseSourceHallToObserver,
            PhaseSource::EncoderToObserver { .. } => Self::PhaseSourceEncoderToObserver,
            PhaseSource::HfiToObserver { .. } => Self::PhaseSourceHfiToObserver,
            PhaseSource::HfiToHall { .. } => Self::PhaseSourceHfiToHall,
            PhaseSource::HfiToEncoder { .. } => Self::PhaseSourceHfiToEncoder,
            PhaseSource::Manual => Self::PhaseSourceManual,
            PhaseSource::OpenLoop => Self::PhaseSourceOpenLoop,
            PhaseSource::HfiToObserverVolts { .. } => Self::PhaseSourceHfiToObserverVolts,
        }
    }
}

impl PhaseSource {
    /// Convert a `PhaseSource` to a protobuf `PhaseSourceRequest`.
    pub fn to_proto_request(self) -> crate::proto::oxifoc::PhaseSourceRequest<'static> {
        let kind: crate::proto::oxifoc::PhaseSourceKind = self.into();
        let mut req = crate::proto::oxifoc::PhaseSourceRequest {
            source: EnumValue::Known(kind),
            blend_low: 0.0,
            blend_high: 0.0,
            min_vel: 0.0,
            min_confidence: 0.0,
            switch_vel: 0.0,
            toggle_v: 0.0,
            unknown_fields: UnknownFields::empty(),
        };
        match self {
            Self::HallToObserver {
                blend_low,
                blend_high,
            }
            | Self::EncoderToObserver {
                blend_low,
                blend_high,
            } => {
                req.blend_low = blend_low;
                req.blend_high = blend_high;
            }
            Self::HfiToObserver {
                min_vel,
                min_confidence,
            } => {
                req.min_vel = min_vel;
                req.min_confidence = min_confidence;
            }
            Self::HfiToHall { switch_vel } | Self::HfiToEncoder { switch_vel } => {
                req.switch_vel = switch_vel;
            }
            Self::HfiToObserverVolts {
                toggle_v,
                min_confidence,
            } => {
                req.toggle_v = toggle_v;
                req.min_confidence = min_confidence;
            }
            Self::Hall | Self::Encoder | Self::Observer | Self::Hfi | Self::Manual | Self::OpenLoop => {}
        }
        req
    }

    /// Decode a `PhaseSource` from a protobuf `PhaseSourceRequest`.
    pub fn from_proto_request(req: &crate::proto::oxifoc::PhaseSourceRequest<'_>) -> Option<Self> {
        use crate::proto::oxifoc::PhaseSourceKind;
        let EnumValue::Known(kind) = req.source else {
            return None;
        };
        let src = match kind {
            PhaseSourceKind::PhaseSourceHall => Self::Hall,
            PhaseSourceKind::PhaseSourceEncoder => Self::Encoder,
            PhaseSourceKind::PhaseSourceObserver => Self::Observer,
            PhaseSourceKind::PhaseSourceHfi => Self::Hfi,
            PhaseSourceKind::PhaseSourceHallToObserver => Self::HallToObserver {
                blend_low: req.blend_low,
                blend_high: req.blend_high,
            },
            PhaseSourceKind::PhaseSourceEncoderToObserver => Self::EncoderToObserver {
                blend_low: req.blend_low,
                blend_high: req.blend_high,
            },
            PhaseSourceKind::PhaseSourceHfiToObserver => Self::HfiToObserver {
                min_vel: req.min_vel,
                min_confidence: req.min_confidence,
            },
            PhaseSourceKind::PhaseSourceHfiToHall => Self::HfiToHall {
                switch_vel: req.switch_vel,
            },
            PhaseSourceKind::PhaseSourceHfiToEncoder => Self::HfiToEncoder {
                switch_vel: req.switch_vel,
            },
            PhaseSourceKind::PhaseSourceManual => Self::Manual,
            PhaseSourceKind::PhaseSourceOpenLoop => Self::OpenLoop,
            PhaseSourceKind::PhaseSourceHfiToObserverVolts => Self::HfiToObserverVolts {
                toggle_v: req.toggle_v,
                min_confidence: req.min_confidence,
            },
        };
        src.is_finite().then_some(src)
    }
}

impl From<ControlMode> for crate::proto::oxifoc::ControlModeMessage<'static> {
    fn from(mode: ControlMode) -> Self {
        use crate::proto::oxifoc::{
            Brake, Coast, ControlModeMessage, CurrentControl, DirectVoltageControl,
            OpenLoopControl, PositionControl, Stopped, VelocityControl, control_mode_message::Mode,
        };
        let proto_mode = match mode {
            ControlMode::Stopped => Mode::Stopped(Stopped {
                unknown_fields: UnknownFields::empty(),
            }),
            ControlMode::Coast => Mode::Coast(Coast {
                unknown_fields: UnknownFields::empty(),
            }),
            ControlMode::Brake => Mode::Brake(Brake {
                unknown_fields: UnknownFields::empty(),
            }),
            ControlMode::CurrentControl {
                iq_target,
                id_target,
            } => Mode::CurrentControl(CurrentControl {
                iq_target,
                id_target,
                unknown_fields: UnknownFields::empty(),
            }),
            ControlMode::VelocityControl { target_vel } => Mode::VelocityControl(VelocityControl {
                target_vel,
                unknown_fields: UnknownFields::empty(),
            }),
            ControlMode::PositionControl { target_pos } => Mode::PositionControl(PositionControl {
                target_pos,
                unknown_fields: UnknownFields::empty(),
            }),
            ControlMode::OpenLoop {
                angle_rad,
                current,
                velocity_rad_s,
                pi_gains,
            } => {
                let (kp, ki, has_pi_gains) = match pi_gains {
                    Some((kp, ki)) => (kp, ki, true),
                    None => (0.0, 0.0, false),
                };
                Mode::OpenLoop(OpenLoopControl {
                    angle_rad,
                    current,
                    velocity_rad_s,
                    kp,
                    ki,
                    has_pi_gains,
                    unknown_fields: UnknownFields::empty(),
                })
            }
            ControlMode::DirectVoltage { vd, vq, angle_rad } => {
                Mode::DirectVoltage(DirectVoltageControl {
                    vd,
                    vq,
                    angle_rad,
                    unknown_fields: UnknownFields::empty(),
                })
            }
        };
        ControlModeMessage {
            mode: Some(proto_mode),
            unknown_fields: UnknownFields::empty(),
        }
    }
}

impl ControlMode {
    /// Decode a `ControlMode` from a protobuf `ControlModeMessage`.
    pub fn from_proto(msg: &crate::proto::oxifoc::ControlModeMessage<'_>) -> Option<Self> {
        use crate::proto::oxifoc::control_mode_message::Mode;
        let mode = match msg.mode.as_ref()? {
            Mode::Stopped(_) => Self::Stopped,
            Mode::Coast(_) => Self::Coast,
            Mode::Brake(_) => Self::Brake,
            Mode::CurrentControl(c) => Self::CurrentControl {
                iq_target: c.iq_target,
                id_target: c.id_target,
            },
            Mode::VelocityControl(v) => Self::VelocityControl {
                target_vel: v.target_vel,
            },
            Mode::PositionControl(p) => Self::PositionControl {
                target_pos: p.target_pos,
            },
            Mode::OpenLoop(o) => Self::OpenLoop {
                angle_rad: o.angle_rad,
                current: o.current,
                velocity_rad_s: o.velocity_rad_s,
                pi_gains: o.has_pi_gains.then_some((o.kp, o.ki)),
            },
            Mode::DirectVoltage(d) => Self::DirectVoltage {
                vd: d.vd,
                vq: d.vq,
                angle_rad: d.angle_rad,
            },
            _ => return None,
        };
        mode.is_finite().then_some(mode)
    }
}

impl From<SlowTelemetry> for crate::proto::oxifoc::SlowTelemetryMessage<'static> {
    fn from(t: SlowTelemetry) -> Self {
        Self {
            vbus_mv: t.vbus_mv,
            fet_temp_c_x10: i32::from(t.fet_temp_c_x10),
            motor_temp_c_x10: i32::from(t.motor_temp_c_x10),
            board_temp_c_x10: i32::from(t.board_temp_c_x10),
            motor_state: EnumValue::Known(t.motor_state.into()),
            control_mode: Some(t.control_mode.into()),
            fault_count: u32::from(t.fault_count),
            phase_source: EnumValue::Known(t.phase_source.into()),
            seq: t.seq,
            derate_drive_pct: u32::from(t.derate_drive_pct),
            derate_brake_pct: u32::from(t.derate_brake_pct),
            fault_generation: t.fault_generation,
            derate_regen_pct: u32::from(t.derate_regen_pct),
            unknown_fields: UnknownFields::empty(),
        }
    }
}

impl From<&crate::proto::oxifoc::SlowTelemetryMessage<'_>> for SlowTelemetry {
    fn from(m: &crate::proto::oxifoc::SlowTelemetryMessage<'_>) -> Self {
        let motor_state = match m.motor_state {
            EnumValue::Known(s) => s.into(),
            EnumValue::Unknown(_) => MotorState::Stopped,
        };
        let control_mode = m
            .control_mode
            .as_ref()
            .and_then(ControlMode::from_proto)
            .unwrap_or(ControlMode::Stopped);
        let phase_source = match m.phase_source {
            EnumValue::Known(crate::proto::oxifoc::PhaseSourceKind::PhaseSourceEncoder) => {
                PhaseSource::Encoder
            }
            EnumValue::Known(crate::proto::oxifoc::PhaseSourceKind::PhaseSourceObserver) => {
                PhaseSource::Observer
            }
            EnumValue::Known(crate::proto::oxifoc::PhaseSourceKind::PhaseSourceHfi) => {
                PhaseSource::Hfi
            }
            EnumValue::Known(crate::proto::oxifoc::PhaseSourceKind::PhaseSourceManual) => {
                PhaseSource::Manual
            }
            EnumValue::Known(crate::proto::oxifoc::PhaseSourceKind::PhaseSourceOpenLoop) => {
                PhaseSource::OpenLoop
            }
            _ => PhaseSource::Hall,
        };
        Self {
            vbus_mv: m.vbus_mv,
            fet_temp_c_x10: m.fet_temp_c_x10 as i16,
            motor_temp_c_x10: m.motor_temp_c_x10 as i16,
            board_temp_c_x10: m.board_temp_c_x10 as i16,
            motor_state,
            control_mode,
            fault_count: m.fault_count as u8,
            phase_source,
            seq: m.seq,
            derate_drive_pct: m.derate_drive_pct as u8,
            derate_brake_pct: m.derate_brake_pct as u8,
            fault_generation: m.fault_generation,
            derate_regen_pct: m.derate_regen_pct as u8,
        }
    }
}

impl From<FastTelemetry> for crate::proto::oxifoc::FastTelemetryMessage<'static> {
    fn from(t: FastTelemetry) -> Self {
        Self {
            ia: u32::from(t.ia),
            ib: u32::from(t.ib),
            ic: u32::from(t.ic),
            vbus: u32::from(t.vbus),
            angle: u32::from(t.angle),
            vd: i32::from(t.vd),
            vq: i32::from(t.vq),
            rpm: i32::from(t.rpm),
            seq: u32::from(t.seq),
            unknown_fields: UnknownFields::empty(),
        }
    }
}

impl From<&crate::proto::oxifoc::FastTelemetryMessage<'_>> for FastTelemetry {
    fn from(m: &crate::proto::oxifoc::FastTelemetryMessage<'_>) -> Self {
        Self {
            ia: m.ia as u16,
            ib: m.ib as u16,
            ic: m.ic as u16,
            vbus: m.vbus as u16,
            angle: m.angle as u16,
            vd: m.vd as i16,
            vq: m.vq as i16,
            rpm: m.rpm as i16,
            seq: m.seq as u16,
        }
    }
}

impl From<FaultStateSnapshot> for crate::proto::oxifoc::FaultStateMessage<'static> {
    fn from(s: FaultStateSnapshot) -> Self {
        Self {
            generation: s.generation,
            active_categories_mask: s.active_categories_mask,
            total: u32::from(s.total),
            primary_category: EnumValue::Known(s.primary_category.into()),
            max_severity: EnumValue::Known(s.max_severity.into()),
            unknown_fields: UnknownFields::empty(),
        }
    }
}

impl From<&crate::proto::oxifoc::FaultStateMessage<'_>> for FaultStateSnapshot {
    fn from(m: &crate::proto::oxifoc::FaultStateMessage<'_>) -> Self {
        let primary_category = match m.primary_category {
            EnumValue::Known(c) => c.into(),
            EnumValue::Unknown(_) => FaultCategory::None,
        };
        let max_severity = match m.max_severity {
            EnumValue::Known(s) => s.into(),
            EnumValue::Unknown(_) => FaultSeverity::Warning,
        };
        Self {
            generation: m.generation,
            active_categories_mask: m.active_categories_mask,
            total: m.total as u8,
            primary_category,
            max_severity,
        }
    }
}

impl From<MotorRequest> for crate::proto::oxifoc::MotorCommandRequest<'static> {
    fn from(req: MotorRequest) -> Self {
        match req.command {
            MotorCommand::EmergencyStop => Self {
                source_session: req.source_session,
                seq: req.seq,
                emergency_stop: true,
                mode: None,
                unknown_fields: UnknownFields::empty(),
            },
            MotorCommand::SetMode(mode) => Self {
                source_session: req.source_session,
                seq: req.seq,
                emergency_stop: false,
                mode: Some(mode.into()),
                unknown_fields: UnknownFields::empty(),
            },
        }
    }
}

impl MotorRequest {
    /// Decode a `MotorRequest` from a protobuf `MotorCommandRequest`.
    pub fn from_proto(msg: &crate::proto::oxifoc::MotorCommandRequest<'_>) -> Option<Self> {
        let command = if msg.emergency_stop {
            MotorCommand::EmergencyStop
        } else {
            let mode = ControlMode::from_proto(msg.mode.as_ref()?)?;
            MotorCommand::SetMode(mode)
        };
        Some(Self {
            source_session: msg.source_session,
            seq: msg.seq,
            command,
        })
    }
}
