/// Stopped mode (PWM disabled).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct Stopped<'a> {
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Coast mode (all FETs high-Z).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct Coast<'a> {
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Viscous brake mode (all low-side FETs on).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct Brake<'a> {
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Current (torque) control mode.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct CurrentControl<'a> {
    #[femtopb(float, tag = 1)]
    pub iq_target: f32,
    #[femtopb(float, tag = 2)]
    pub id_target: f32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Velocity control mode.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct VelocityControl<'a> {
    #[femtopb(float, tag = 1)]
    pub target_vel: f32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Position control mode.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct PositionControl<'a> {
    #[femtopb(float, tag = 1)]
    pub target_pos: f32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Open-loop control mode.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct OpenLoopControl<'a> {
    #[femtopb(float, tag = 1)]
    pub angle_rad: f32,
    #[femtopb(float, tag = 2)]
    pub current: f32,
    #[femtopb(float, tag = 3)]
    pub velocity_rad_s: f32,
    #[femtopb(float, tag = 4)]
    pub kp: f32,
    #[femtopb(float, tag = 5)]
    pub ki: f32,
    #[femtopb(bool, tag = 6)]
    pub has_pi_gains: bool,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Direct dq voltage mode.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct DirectVoltageControl<'a> {
    #[femtopb(float, tag = 1)]
    pub vd: f32,
    #[femtopb(float, tag = 2)]
    pub vq: f32,
    #[femtopb(float, tag = 3)]
    pub angle_rad: f32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Motor control mode union.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct ControlModeMessage<'a> {
    #[femtopb(oneof, tags = [1, 2, 3, 4, 5, 6, 7, 8])]
    pub mode: ::core::option::Option<control_mode_message::Mode<'a>>,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Nested message and enum types in `ControlModeMessage`.
pub mod control_mode_message {
    #[derive(Clone, Copy, PartialEq, ::femtopb::Oneof)]
    #[non_exhaustive]
    pub enum Mode<'a> {
        #[femtopb(message, tag = 1)]
        Stopped(super::Stopped<'a>),
        #[femtopb(message, tag = 2)]
        CurrentControl(super::CurrentControl<'a>),
        #[femtopb(message, tag = 3)]
        VelocityControl(super::VelocityControl<'a>),
        #[femtopb(message, tag = 4)]
        PositionControl(super::PositionControl<'a>),
        #[femtopb(message, tag = 5)]
        OpenLoop(super::OpenLoopControl<'a>),
        #[femtopb(message, tag = 6)]
        DirectVoltage(super::DirectVoltageControl<'a>),
        #[femtopb(message, tag = 7)]
        Coast(super::Coast<'a>),
        #[femtopb(message, tag = 8)]
        Brake(super::Brake<'a>),
        #[femtopb(phantom)]
        _Phantom(::core::marker::PhantomData<&'a ()>),
    }
}
/// Low-frequency system telemetry (device -> host).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct SlowTelemetryMessage<'a> {
    #[femtopb(uint32, tag = 1)]
    pub vbus_mv: u32,
    #[femtopb(int32, tag = 2)]
    pub fet_temp_c_x10: i32,
    #[femtopb(int32, tag = 3)]
    pub motor_temp_c_x10: i32,
    #[femtopb(int32, tag = 4)]
    pub board_temp_c_x10: i32,
    #[femtopb(enumeration, tag = 5)]
    pub motor_state: ::femtopb::enumeration::EnumValue<MotorState>,
    #[femtopb(message, optional, tag = 6)]
    pub control_mode: ::core::option::Option<ControlModeMessage<'a>>,
    #[femtopb(uint32, tag = 7)]
    pub fault_count: u32,
    #[femtopb(enumeration, tag = 8)]
    pub phase_source: ::femtopb::enumeration::EnumValue<PhaseSourceKind>,
    #[femtopb(uint32, tag = 9)]
    pub seq: u32,
    #[femtopb(uint32, tag = 10)]
    pub derate_drive_pct: u32,
    #[femtopb(uint32, tag = 11)]
    pub derate_brake_pct: u32,
    #[femtopb(uint32, tag = 12)]
    pub fault_generation: u32,
    #[femtopb(uint32, tag = 13)]
    pub derate_regen_pct: u32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// High-frequency raw-ADC + FOC diagnostic telemetry (device -> host).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct FastTelemetryMessage<'a> {
    #[femtopb(uint32, tag = 1)]
    pub ia: u32,
    #[femtopb(uint32, tag = 2)]
    pub ib: u32,
    #[femtopb(uint32, tag = 3)]
    pub ic: u32,
    #[femtopb(uint32, tag = 4)]
    pub vbus: u32,
    #[femtopb(uint32, tag = 5)]
    pub angle: u32,
    #[femtopb(int32, tag = 6)]
    pub vd: i32,
    #[femtopb(int32, tag = 7)]
    pub vq: i32,
    #[femtopb(int32, tag = 8)]
    pub rpm: i32,
    #[femtopb(uint32, tag = 9)]
    pub seq: u32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Active fault registry snapshot (device -> host).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct FaultStateMessage<'a> {
    #[femtopb(uint32, tag = 1)]
    pub generation: u32,
    #[femtopb(uint32, tag = 2)]
    pub active_categories_mask: u32,
    #[femtopb(uint32, tag = 3)]
    pub total: u32,
    #[femtopb(enumeration, tag = 4)]
    pub primary_category: ::femtopb::enumeration::EnumValue<FaultCategory>,
    #[femtopb(enumeration, tag = 5)]
    pub max_severity: ::femtopb::enumeration::EnumValue<FaultSeverity>,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Sequenced motor command request (host -> device).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct MotorCommandRequest<'a> {
    #[femtopb(uint64, tag = 1)]
    pub source_session: u64,
    #[femtopb(uint32, tag = 2)]
    pub seq: u32,
    #[femtopb(bool, tag = 3)]
    pub emergency_stop: bool,
    #[femtopb(message, optional, tag = 4)]
    pub mode: ::core::option::Option<ControlModeMessage<'a>>,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Fault clear request (host -> device).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct FaultClearRequest<'a> {
    #[femtopb(uint32, tag = 1)]
    pub expected_generation: u32,
    #[femtopb(bool, tag = 2)]
    pub clear_all: bool,
    #[femtopb(enumeration, tag = 3)]
    pub category: ::femtopb::enumeration::EnumValue<FaultCategory>,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Fast telemetry rate configuration request (host -> device).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct TelemetryConfigRequest<'a> {
    #[femtopb(uint32, tag = 1)]
    pub fast_hz: u32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Phase source switch request (host -> device).
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct PhaseSourceRequest<'a> {
    #[femtopb(enumeration, tag = 1)]
    pub source: ::femtopb::enumeration::EnumValue<PhaseSourceKind>,
    #[femtopb(float, tag = 2)]
    pub blend_low: f32,
    #[femtopb(float, tag = 3)]
    pub blend_high: f32,
    #[femtopb(float, tag = 4)]
    pub min_vel: f32,
    #[femtopb(float, tag = 5)]
    pub min_confidence: f32,
    #[femtopb(float, tag = 6)]
    pub switch_vel: f32,
    #[femtopb(float, tag = 7)]
    pub toggle_v: f32,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Top-level wire message encoded with streaming-protocol and fragmented via NMEA over CAN.
#[derive(Clone, Copy, PartialEq, ::femtopb::Message)]
pub struct WireMessage<'a> {
    #[femtopb(oneof, tags = [1, 2, 3, 10, 11, 12, 13])]
    pub payload: ::core::option::Option<wire_message::Payload<'a>>,
    #[femtopb(unknown_fields)]
    pub unknown_fields: femtopb::UnknownFields<'a>,
}
/// Nested message and enum types in `WireMessage`.
pub mod wire_message {
    #[derive(Clone, Copy, PartialEq, ::femtopb::Oneof)]
    #[non_exhaustive]
    pub enum Payload<'a> {
        /// Device -> Host telemetry
        #[femtopb(message, tag = 1)]
        SlowTelemetry(super::SlowTelemetryMessage<'a>),
        #[femtopb(message, tag = 2)]
        FastTelemetry(super::FastTelemetryMessage<'a>),
        #[femtopb(message, tag = 3)]
        FaultState(super::FaultStateMessage<'a>),
        /// Host -> Device commands
        #[femtopb(message, tag = 10)]
        MotorCommand(super::MotorCommandRequest<'a>),
        #[femtopb(message, tag = 11)]
        FaultClear(super::FaultClearRequest<'a>),
        #[femtopb(message, tag = 12)]
        TelemetryConfig(super::TelemetryConfigRequest<'a>),
        #[femtopb(message, tag = 13)]
        PhaseSourceRequest(super::PhaseSourceRequest<'a>),
        #[femtopb(phantom)]
        _Phantom(::core::marker::PhantomData<&'a ()>),
    }
}
/// Operational state of the motor controller.
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Hash,
    PartialOrd,
    Ord,
    ::femtopb::Enumeration
)]
#[repr(i32)]
#[derive(Default)]
pub enum MotorState {
    #[default]
    Stopped = 0,
    Running = 1,
    Error = 2,
}
impl MotorState {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            Self::Stopped => "MOTOR_STATE_STOPPED",
            Self::Running => "MOTOR_STATE_RUNNING",
            Self::Error => "MOTOR_STATE_ERROR",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "MOTOR_STATE_STOPPED" => Some(Self::Stopped),
            "MOTOR_STATE_RUNNING" => Some(Self::Running),
            "MOTOR_STATE_ERROR" => Some(Self::Error),
            _ => None,
        }
    }
}
/// Protocol-level fault categories.
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Hash,
    PartialOrd,
    Ord,
    ::femtopb::Enumeration
)]
#[repr(i32)]
#[derive(Default)]
pub enum FaultCategory {
    #[default]
    None = 0,
    OverCurrent = 1,
    OverVoltage = 2,
    UnderVoltage = 3,
    OverTemp = 4,
    DriverFault = 5,
    HallError = 6,
    Stall = 7,
    CalibrationFault = 8,
    CommTimeout = 9,
    Derating = 10,
}
impl FaultCategory {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            Self::None => "FAULT_CATEGORY_NONE",
            Self::OverCurrent => "FAULT_CATEGORY_OVER_CURRENT",
            Self::OverVoltage => "FAULT_CATEGORY_OVER_VOLTAGE",
            Self::UnderVoltage => "FAULT_CATEGORY_UNDER_VOLTAGE",
            Self::OverTemp => "FAULT_CATEGORY_OVER_TEMP",
            Self::DriverFault => "FAULT_CATEGORY_DRIVER_FAULT",
            Self::HallError => "FAULT_CATEGORY_HALL_ERROR",
            Self::Stall => "FAULT_CATEGORY_STALL",
            Self::CalibrationFault => "FAULT_CATEGORY_CALIBRATION_FAULT",
            Self::CommTimeout => "FAULT_CATEGORY_COMM_TIMEOUT",
            Self::Derating => "FAULT_CATEGORY_DERATING",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "FAULT_CATEGORY_NONE" => Some(Self::None),
            "FAULT_CATEGORY_OVER_CURRENT" => Some(Self::OverCurrent),
            "FAULT_CATEGORY_OVER_VOLTAGE" => Some(Self::OverVoltage),
            "FAULT_CATEGORY_UNDER_VOLTAGE" => Some(Self::UnderVoltage),
            "FAULT_CATEGORY_OVER_TEMP" => Some(Self::OverTemp),
            "FAULT_CATEGORY_DRIVER_FAULT" => Some(Self::DriverFault),
            "FAULT_CATEGORY_HALL_ERROR" => Some(Self::HallError),
            "FAULT_CATEGORY_STALL" => Some(Self::Stall),
            "FAULT_CATEGORY_CALIBRATION_FAULT" => Some(Self::CalibrationFault),
            "FAULT_CATEGORY_COMM_TIMEOUT" => Some(Self::CommTimeout),
            "FAULT_CATEGORY_DERATING" => Some(Self::Derating),
            _ => None,
        }
    }
}
/// Severity response class for a fault.
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Hash,
    PartialOrd,
    Ord,
    ::femtopb::Enumeration
)]
#[repr(i32)]
#[derive(Default)]
pub enum FaultSeverity {
    #[default]
    Warning = 0,
    GracefulStop = 1,
    Kill = 2,
}
impl FaultSeverity {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            Self::Warning => "FAULT_SEVERITY_WARNING",
            Self::GracefulStop => "FAULT_SEVERITY_GRACEFUL_STOP",
            Self::Kill => "FAULT_SEVERITY_KILL",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "FAULT_SEVERITY_WARNING" => Some(Self::Warning),
            "FAULT_SEVERITY_GRACEFUL_STOP" => Some(Self::GracefulStop),
            "FAULT_SEVERITY_KILL" => Some(Self::Kill),
            _ => None,
        }
    }
}
/// Electrical phase source selection.
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Hash,
    PartialOrd,
    Ord,
    ::femtopb::Enumeration
)]
#[repr(i32)]
#[derive(Default)]
pub enum PhaseSourceKind {
    #[default]
    PhaseSourceHall = 0,
    PhaseSourceEncoder = 1,
    PhaseSourceObserver = 2,
    PhaseSourceHfi = 3,
    PhaseSourceHallToObserver = 4,
    PhaseSourceEncoderToObserver = 5,
    PhaseSourceHfiToObserver = 6,
    PhaseSourceHfiToHall = 7,
    PhaseSourceHfiToEncoder = 8,
    PhaseSourceManual = 9,
    PhaseSourceOpenLoop = 10,
    PhaseSourceHfiToObserverVolts = 11,
}
impl PhaseSourceKind {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            Self::PhaseSourceHall => "PHASE_SOURCE_HALL",
            Self::PhaseSourceEncoder => "PHASE_SOURCE_ENCODER",
            Self::PhaseSourceObserver => "PHASE_SOURCE_OBSERVER",
            Self::PhaseSourceHfi => "PHASE_SOURCE_HFI",
            Self::PhaseSourceHallToObserver => "PHASE_SOURCE_HALL_TO_OBSERVER",
            Self::PhaseSourceEncoderToObserver => "PHASE_SOURCE_ENCODER_TO_OBSERVER",
            Self::PhaseSourceHfiToObserver => "PHASE_SOURCE_HFI_TO_OBSERVER",
            Self::PhaseSourceHfiToHall => "PHASE_SOURCE_HFI_TO_HALL",
            Self::PhaseSourceHfiToEncoder => "PHASE_SOURCE_HFI_TO_ENCODER",
            Self::PhaseSourceManual => "PHASE_SOURCE_MANUAL",
            Self::PhaseSourceOpenLoop => "PHASE_SOURCE_OPEN_LOOP",
            Self::PhaseSourceHfiToObserverVolts => "PHASE_SOURCE_HFI_TO_OBSERVER_VOLTS",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "PHASE_SOURCE_HALL" => Some(Self::PhaseSourceHall),
            "PHASE_SOURCE_ENCODER" => Some(Self::PhaseSourceEncoder),
            "PHASE_SOURCE_OBSERVER" => Some(Self::PhaseSourceObserver),
            "PHASE_SOURCE_HFI" => Some(Self::PhaseSourceHfi),
            "PHASE_SOURCE_HALL_TO_OBSERVER" => Some(Self::PhaseSourceHallToObserver),
            "PHASE_SOURCE_ENCODER_TO_OBSERVER" => {
                Some(Self::PhaseSourceEncoderToObserver)
            }
            "PHASE_SOURCE_HFI_TO_OBSERVER" => Some(Self::PhaseSourceHfiToObserver),
            "PHASE_SOURCE_HFI_TO_HALL" => Some(Self::PhaseSourceHfiToHall),
            "PHASE_SOURCE_HFI_TO_ENCODER" => Some(Self::PhaseSourceHfiToEncoder),
            "PHASE_SOURCE_MANUAL" => Some(Self::PhaseSourceManual),
            "PHASE_SOURCE_OPEN_LOOP" => Some(Self::PhaseSourceOpenLoop),
            "PHASE_SOURCE_HFI_TO_OBSERVER_VOLTS" => {
                Some(Self::PhaseSourceHfiToObserverVolts)
            }
            _ => None,
        }
    }
}
