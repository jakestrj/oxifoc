//! Shared types for motor control ICD (Interface Control Document)
//!
//! This module contains types that are shared between embedded firmware and
//! host applications. All types here are serializable via serde/postcard.
//!
//! # Feature: `types`
//!
//! This module is only available when the `types` feature is enabled.
//! Host applications that only need type definitions can use:
//! ```toml
//! oxifoc-core = { version = "0.1", default-features = false, features = ["types"] }
//! ```

use crate::foc::phase::PhaseSource;
use heapless::String;
use heapless::Vec;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// Re-export Direction from hall_sensor (it has conditional serde derives)
pub use crate::foc::hall_sensor::Direction;

// Re-export fault types for protocol use
pub use crate::foc::current_offset::{CurrentOffsetMethod, CurrentOffsetReport};
pub use crate::foc::fault::{FaultCategory, FaultInfo};

// ============================================================================
// Delivery / idempotency-key wire types
// ============================================================================

/// A client-chosen request id, stable across retries of the same logical
/// request. The server deduplicates on it so a non-idempotent action runs at
/// most once. See the `delivery` module for the full ladder.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReqId(pub u64);

/// A request payload tagged with a [`ReqId`] — the wire envelope for a
/// deduplicated (effectively-once) endpoint. Mirrors an HTTP idempotency key.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Keyed<T> {
    /// Stable across retries; the dedup key.
    pub id: ReqId,
    /// The actual request.
    pub inner: T,
}

impl<T> Keyed<T> {
    /// Tag a request with an id.
    pub fn new(id: ReqId, inner: T) -> Self {
        Self { id, inner }
    }
}

// ============================================================================
// Motor State Types
// ============================================================================

/// Motor operational state
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorState {
    /// Motor is stopped (PWM disabled or zero duty)
    #[default]
    Stopped,
    /// Motor is running (FOC active)
    Running,
    /// Motor is in error state (fault detected)
    Error,
}

/// Control mode for the motor
///
/// This is the unified control mode used by both the protocol layer and the FOC driver.
/// Send this directly via the command channel to control the motor.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlMode {
    /// Motor stopped, PWM disabled
    #[default]
    Stopped,
    /// Current control mode (torque control)
    CurrentControl {
        /// Target q-axis current (torque) in Amps
        iq_target: f32,
        /// Target d-axis current (field weakening) in Amps
        id_target: f32,
    },
    /// Velocity control mode
    VelocityControl {
        /// Target velocity in rad/s
        target_vel: f32,
    },
    /// Position control mode
    PositionControl {
        /// Target position in radians
        target_pos: f32,
    },
    /// Open-loop mode — drive motor at commanded electrical angle.
    ///
    /// Uses commanded angle instead of sensor feedback. Current control still
    /// runs to regulate the applied current.
    ///
    /// When `velocity_rad_s == 0`: locks rotor at `angle_rad` (calibration use).
    /// When `velocity_rad_s != 0`: firmware advances angle at the given speed
    /// (open-loop spinning without hall sensors).
    OpenLoop {
        /// Initial electrical angle (radians, 0 to 2π)
        angle_rad: f32,
        /// Current magnitude (Amps) - applied as q-current
        current: f32,
        /// Electrical velocity (rad/s) — 0 = lock, nonzero = spin
        velocity_rad_s: f32,
        /// Optional PI gains override (kp, ki). Applied on mode entry.
        /// Used by detection to set conservative gains when motor params are unknown.
        pi_gains: Option<(f32, f32)>,
    },
    /// Direct voltage mode — apply dq voltages without PI control.
    ///
    /// Bypasses current regulation entirely. Used for measurement modes
    /// (HFI inductance detection), calibration, and board bringup where
    /// PI interference is undesirable.
    DirectVoltage {
        /// d-axis voltage (V)
        vd: f32,
        /// q-axis voltage (V)
        vq: f32,
        /// Electrical angle (radians)
        angle_rad: f32,
    },
    /// Coast mode — all FETs off (high-impedance), motor spins freely.
    ///
    /// Used during spin-down flux linkage measurement.  Phase voltages
    /// float so back-EMF can be read directly by ADC.
    Coast,
    /// Brake mode — all low-side FETs on, windings shorted (parking brake).
    ///
    /// Resists motion with speed-proportional torque and dissipates the
    /// energy in the motor (no regen, no bus pumping); draws nothing at
    /// standstill, so it can be held indefinitely. It is a *viscous* brake,
    /// not a position hold — on a slope the board creeps slowly.
    ///
    /// Entry is speed-gated at the command boundary: shorting the windings
    /// at speed dumps an uncontrolled current (~λ/L) through the FETs, so
    /// `process_commands` rejects it above a small velocity threshold.
    Brake,
}

// ============================================================================
// Status/Telemetry Types
// ============================================================================

/// Motor status response
///
/// Note: Fault information is now platform-specific. Use the platform's
/// fault endpoint to get detailed fault information.
impl ControlMode {
    /// All f32 payload fields are finite.
    ///
    /// Wire input is arbitrary bits: a NaN/inf target doesn't panic (float
    /// math never does), but it propagates through the PI loop into a
    /// garbage SVPWM vector. Commands failing this check must be dropped
    /// at the boundary.
    pub fn is_finite(&self) -> bool {
        match *self {
            Self::Stopped | Self::Coast | Self::Brake => true,
            Self::CurrentControl {
                iq_target,
                id_target,
            } => iq_target.is_finite() && id_target.is_finite(),
            Self::VelocityControl { target_vel } => target_vel.is_finite(),
            Self::PositionControl { target_pos } => target_pos.is_finite(),
            Self::OpenLoop {
                angle_rad,
                current,
                velocity_rad_s,
                pi_gains,
            } => {
                angle_rad.is_finite()
                    && current.is_finite()
                    && velocity_rad_s.is_finite()
                    && pi_gains.is_none_or(|(kp, ki)| kp.is_finite() && ki.is_finite())
            }
            Self::DirectVoltage { vd, vq, angle_rad } => {
                vd.is_finite() && vq.is_finite() && angle_rad.is_finite()
            }
        }
    }

    /// Whether the bridge is high-impedance (all FETs off, phases floating) in
    /// this mode. In a high-Z state the phase terminals show the motor's
    /// back-EMF, so a board with phase sensing can feed the *measured* terminal
    /// voltage to the observer (coasting-rotation tracking / flying start).
    ///
    /// `Brake` is deliberately excluded: it shorts the low sides (terminals ≈
    /// 0 V, not floating), so it keeps its own `(0 V, measured-i)` observer
    /// feed rather than the measured-BEMF path.
    pub fn is_high_z(&self) -> bool {
        matches!(self, Self::Stopped | Self::Coast)
    }
}

/// One host/remote process generation. A new process chooses a fresh value;
/// sequence numbers are compared only within that generation.
pub type DriveSessionId = u64;

/// Action carried by the existing motor endpoint.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorCommand {
    /// Apply an absolute control mode/setpoint.
    SetMode(ControlMode),
    /// Immediately float the bridge and latch re-arm until a subsequent safe
    /// neutral command (Stopped/Coast/Brake).
    EmergencyStop,
}

impl MotorCommand {
    pub fn is_finite(self) -> bool {
        match self {
            Self::SetMode(mode) => mode.is_finite(),
            Self::EmergencyStop => true,
        }
    }
}

/// Sequenced request for [`crate::icd::MotorEndpoint`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorRequest {
    pub source_session: DriveSessionId,
    pub seq: u32,
    pub command: MotorCommand,
}

/// Result produced by the ISR command gate.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorCommandOutcome {
    /// The action was applied by the ISR.
    #[default]
    Applied,
    /// This exact source/sequence had already been applied; it was not applied
    /// again and did not refresh the command deadman.
    AlreadyApplied,
    /// Sequence is older than the last accepted value for this source.
    RejectedStale,
    /// A different source currently owns active drive control.
    RejectedSourceBusy,
    /// A fault, calibration, flash operation, or re-arm latch blocked it.
    RejectedSafety,
    /// Payload failed validation.
    RejectedInvalid,
    // postcard encodes the variant index — append only.
    /// The ISR did not confirm the command within the ack window (control
    /// loop not running yet, or stalled — e.g. by a flash erase). The
    /// command MAY still be applied once the ISR resumes; retrying with the
    /// same `seq` is safe (a late application answers `AlreadyApplied`).
    TimedOut,
}

impl MotorCommandOutcome {
    pub fn is_success(self) -> bool {
        matches!(self, Self::Applied | Self::AlreadyApplied)
    }
}

/// Device-local ownership/sequence guard for active drive commands. This is
/// deliberately not serialized: both the real ISR and virtual device use the
/// same policy implementation.
#[derive(Clone, Copy, Debug, Default)]
pub struct DriveOwnership {
    owner: Option<DriveOwner>,
}

#[derive(Clone, Copy, Debug)]
struct DriveOwner {
    session: DriveSessionId,
    last_seq: u32,
}

impl DriveOwnership {
    pub const fn new() -> Self {
        Self { owner: None }
    }

    pub fn authorize(self, request: MotorRequest) -> Result<(), MotorCommandOutcome> {
        let MotorCommand::SetMode(mode) = request.command else {
            return Ok(());
        };
        if matches!(
            mode,
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        ) {
            return Ok(());
        }
        let Some(owner) = self.owner else {
            return Ok(());
        };
        if owner.session != request.source_session {
            return Err(MotorCommandOutcome::RejectedSourceBusy);
        }
        if request.seq == owner.last_seq {
            return Err(MotorCommandOutcome::AlreadyApplied);
        }
        let delta = request.seq.wrapping_sub(owner.last_seq);
        if delta >= (1 << 31) {
            return Err(MotorCommandOutcome::RejectedStale);
        }
        Ok(())
    }

    pub fn record_applied(&mut self, request: MotorRequest) {
        self.owner = match request.command {
            MotorCommand::SetMode(mode)
                if !matches!(
                    mode,
                    ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
                ) =>
            {
                Some(DriveOwner {
                    session: request.source_session,
                    last_seq: request.seq,
                })
            }
            MotorCommand::SetMode(_) | MotorCommand::EmergencyStop => None,
        };
    }

    pub fn release(&mut self) {
        self.owner = None;
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorStatus {
    /// Whether this request was applied (or was an idempotent duplicate).
    pub outcome: MotorCommandOutcome,
    /// Current motor state
    pub state: MotorState,
    /// Current control mode
    pub mode: ControlMode,
    /// Number of active faults
    pub fault_count: u8,
}

// ============================================================================
// Streaming Telemetry Types (push-based via Topics)
// ============================================================================

/// High-frequency motor diagnostic telemetry (streamed at configurable rate).
///
/// **Raw-ADC diagnostic frame.** Carries the three phase-current shunt readings
/// as uncalibrated 12-bit ADC counts — no calibration, sign correction, or
/// anti-alias decimation — straight from the converter. The host applies
/// offset/sign/scale (per `BoardConfig`) and reconstructs iα/iβ/id/iq in
/// post-processing. The third shunt is kept (not reconstructed from
/// `−(ia+ib)`) so a per-phase current-sense fault stays visible in the raw
/// data. Frame/field rationale: docs/notes/rtt-telemetry-throughput.md §6.
///
/// `#[repr(C)]`, 9×u16/i16 = **18 bytes**, align 2, no padding → `Pod` clean.
/// The in-memory layout IS the wire layout: batches ship the little-endian
/// Pod bytes verbatim (see [`FastTelemetryBatch`]), so this struct doubles as
/// the wire contract — reorder/resize fields only together with the host.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "pod", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct FastTelemetry {
    /// Phase A current — raw ADC counts (uncalibrated). Host reconstructs amps
    /// via `BoardConfig` (vref/counts/amp_gain/shunt) and the per-phase
    /// zero-current `dc_offsets`; `iα/iβ/id/iq` then follow from Clarke/Park.
    pub ia: u16,
    /// Phase B current — raw ADC counts
    pub ib: u16,
    /// Phase C current — raw ADC counts
    pub ic: u16,
    /// Bus voltage in **2 mV** units (u16 → 0..131 V, covers VESC classes).
    pub vbus: u16,
    /// Electrical angle, full-scale `u16` (0..2π). Needed host-side for Park.
    pub angle: u16,
    /// D-axis applied voltage (PI output) in **2 mV** units (i16 → ±65 V).
    /// Not reconstructable from currents (depends on PI integrator state).
    pub vd: i16,
    /// Q-axis applied voltage (PI output) in **2 mV** units.
    pub vq: i16,
    /// Mechanical speed in **2 RPM** units (i16 → ±65534 RPM). The ACTIVE
    /// angle source's velocity (hall / observer / HFI / startup ramp — see
    /// `FocOutput::velocity_rad_s`); before 2026-07-06 this read the hall
    /// estimator unconditionally and showed 0 on sensorless boards while
    /// spinning. Still hard-0 whenever `pole_pairs` is unknown (0).
    pub rpm: i16,
    /// Sequence number — `u16` (FOC-cycle counter mod 65536). At 20 kHz wraps
    /// every ~3.3 s; loss detection uses `wrapping_sub` (unambiguous for gaps
    /// < 1.6 s); host accumulates deltas for the time axis on longer captures.
    pub seq: u16,
}

/// Samples per fast-telemetry batch. Sized so the encoded batch statically
/// fits the SMALLEST board interface MTU (f405 `MAX_PACKET_SIZE` = 512):
/// 26 × 18 = 468 B payload + [`FAST_BATCH_WIRE_OVERHEAD`]. The previous 32
/// (576 B) exceeded 512 and full batches silently failed to broadcast on the
/// f405 (visible only as `BCAST_FAILS`). Every device crate pins the
/// relationship with a compile-time assert next to its `MAX_PACKET_SIZE`.
/// Batch size does not move throughput on the byte-rate-bound debug links
/// (docs/notes/rtt-telemetry-throughput.md §4.7) — it only has to fit the MTU.
pub const FAST_BATCH_SAMPLES: usize = 26;

/// Byte capacity of a fast-telemetry batch: samples × 18 B Pod frame.
pub const FAST_BATCH_BYTES: usize = FAST_BATCH_SAMPLES * size_of::<FastTelemetry>();

/// Worst-case wire bytes around an encoded batch topic frame: ergot's
/// `MAX_HDR_ENCODED_SIZE` (28, includes the broadcast key appendix) + the
/// postcard varint length prefix (2 for lengths ≤ 16383) + slack.
pub const FAST_BATCH_WIRE_OVERHEAD: usize = 32;

/// Batch of fast telemetry samples for efficient network transmission.
///
/// **Raw-Pod encoding**: `data` holds the concatenated little-endian
/// [`FastTelemetry`] Pod bytes (18 B each), NOT a postcard element sequence.
/// postcard serializes `Vec<u8>` as varint(len) + verbatim bytes, so:
/// - device-side encode is a memcpy (no per-field varint work in the hot
///   telemetry path — measured CPU relief on a core that also runs a 20 kHz
///   FOC ISR);
/// - the wire size is a compile-time constant (18 B/sample regardless of
///   values), so an encoded batch can never straddle the MTU — varint frames
///   grew with motor activity and silently exceeded it (see the BATCH=64 bug,
///   commit e1f65b5);
/// - both ends are little-endian (Cortex-M, x86); the Pod layout is the wire
///   contract, round-trip covered by `batch_roundtrip` below.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastTelemetryBatch {
    /// `18 × n` raw frame bytes for `n` samples, `n ≤ FAST_BATCH_SAMPLES`.
    pub data: Vec<u8, FAST_BATCH_BYTES>,
}

impl FastTelemetryBatch {
    pub const fn new() -> Self {
        Self { data: Vec::new() }
    }

    /// Number of whole samples currently in the batch.
    pub fn len(&self) -> usize {
        self.data.len() / size_of::<FastTelemetry>()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// True when another 18 B frame no longer fits.
    pub fn is_full(&self) -> bool {
        self.data.len() + size_of::<FastTelemetry>() > FAST_BATCH_BYTES
    }

    /// Append one frame's raw Pod bytes. Returns `false` (batch unchanged) if
    /// `frame` is not exactly one 18 B frame or the batch is full.
    pub fn push_bytes(&mut self, frame: &[u8]) -> bool {
        frame.len() == size_of::<FastTelemetry>() && self.data.extend_from_slice(frame).is_ok()
    }

    /// Append one sample (device-side convenience over [`Self::push_bytes`]).
    #[cfg(feature = "pod")]
    pub fn push(&mut self, sample: &FastTelemetry) -> bool {
        self.push_bytes(bytemuck::bytes_of(sample))
    }

    /// Decode the batch back into samples (host-side).
    #[cfg(feature = "pod")]
    pub fn samples(&self) -> impl Iterator<Item = FastTelemetry> + '_ {
        self.data
            .chunks_exact(size_of::<FastTelemetry>())
            .map(bytemuck::pod_read_unaligned)
    }
}

/// Low-frequency system telemetry (streamed at configurable rate, default 10Hz)
///
/// Contains slowly-changing system health data. Firmware pushes this
/// at a lower rate since these values don't change rapidly.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlowTelemetry {
    /// DC bus voltage in millivolts
    pub vbus_mv: u32,
    /// FET temperature in 0.1°C units
    pub fet_temp_c_x10: i16,
    /// Motor temperature in 0.1°C units (0 if not available)
    pub motor_temp_c_x10: i16,
    /// Board temperature in 0.1°C units (0 if not available)
    pub board_temp_c_x10: i16,
    /// Current motor state
    pub motor_state: MotorState,
    /// Current control mode
    pub control_mode: ControlMode,
    /// Number of active faults
    pub fault_count: u8,
    /// Active phase source (Hall / Observer / HFI / crossovers)
    pub phase_source: PhaseSource,
    /// Monotonic sequence number
    pub seq: u32,
    // postcard struct fields are positional — append only.
    /// Live drive-side derating (percent of the configured limit, 100 =
    /// no derate) — "why does the board feel weak" at a glance
    pub derate_drive_pct: u8,
    /// Live brake-side derating (percent, 100 = no derate)
    pub derate_brake_pct: u8,
    /// Fault-registry generation. Lets consumers detect a lost/refined fault
    /// snapshot even when the number of active faults did not change.
    pub fault_generation: u32,
    /// Share of the brake's regen the supply takes (percent, 100 = all):
    /// below 100 the pack is full — the refused share is burned in the
    /// windings (dissipative braking) or, with that off, cut from the
    /// brake torque
    pub derate_regen_pct: u8,
}

// ============================================================================
// Gate Driver Telemetry Types (Infineon EiceDRIVER 1ED38x0Mc12M)
// ============================================================================

/// Identifier for one of the 6 EiceDRIVER gate driver switches in a 3-phase bridge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GateDriverSwitch {
    /// Phase U high-side switch (I2C `0x0E`)
    #[default]
    Uh = 0,
    /// Phase U low-side switch (I2C `0x0F`)
    Ul = 1,
    /// Phase V high-side switch (I2C `0x10`)
    Vh = 2,
    /// Phase V low-side switch (I2C `0x11`)
    Vl = 3,
    /// Phase W high-side switch (I2C `0x12`)
    Wh = 4,
    /// Phase W low-side switch (I2C `0x13`)
    Wl = 5,
}

impl GateDriverSwitch {
    /// Total number of gate driver switches in a 3-phase bridge.
    pub const COUNT: usize = 6;

    /// All 6 gate driver switches in address order (`UH`..`WL`).
    pub const ALL: [Self; Self::COUNT] =
        [Self::Uh, Self::Ul, Self::Vh, Self::Vl, Self::Wh, Self::Wl];

    /// 0-based index (`0..6`).
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Convert from a 0-based switch index (`0..6`).
    #[must_use]
    pub const fn from_index(idx: usize) -> Option<Self> {
        match idx {
            0 => Some(Self::Uh),
            1 => Some(Self::Ul),
            2 => Some(Self::Vh),
            3 => Some(Self::Vl),
            4 => Some(Self::Wh),
            5 => Some(Self::Wl),
            _ => None,
        }
    }

    /// Short human-readable label (`"UH"`, `"UL"`, etc.).
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Uh => "UH",
            Self::Ul => "UL",
            Self::Vh => "VH",
            Self::Vl => "VL",
            Self::Wh => "WH",
            Self::Wl => "WL",
        }
    }

    /// Configured 7-bit I2C address (`0x0E..=0x13`).
    #[must_use]
    pub const fn i2c_addr(self) -> u8 {
        0x0E + (self as u8)
    }
}

/// Telemetry and status registers for a single EiceDRIVER 1ED38x0Mc12M switch.
///
/// Mirrors `gd_values` in `bldc-custom/eicedriver` (`eicedriver.c` / `commands.c`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GateDriverSwitchTelemetry {
    /// Which bridge switch this telemetry belongs to.
    pub switch_id: GateDriverSwitch,
    /// True if all 6 gate driver ICs completed startup I2C enumeration.
    pub configured: bool,
    /// True if the most recent 16-byte I2C status read succeeded.
    pub i2c_ok: bool,
    /// Physical `FLT_N` pin state (`true` = high / no active fault pull-down).
    pub flt_n: bool,
    /// Physical `RDYC` pin state (`true` = high / ready).
    pub rdyc: bool,
    /// `RDYSTAT[0]` — chip ready.
    pub chip_rdy: bool,
    /// `RDYSTAT[1]` — primary-side ready.
    pub pri_rdy: bool,
    /// `RDYSTAT[2]` — secondary-side ready.
    pub sec_rdy: bool,
    /// `SECUVEVT` (`0x1D`) — secondary supply undervoltage event flags.
    pub secuvevt: u8,
    /// `GFLTEVT` (`0x1E`) — gate fault event flags.
    pub gfltevt: u8,
    /// `FLTEVT` (`0x1F`) — primary fault event flags (DESAT, SCP, OTSD, etc.).
    pub fltevt: u8,
    /// `PINSTAT` (`0x20`) — input/output pin status.
    pub pinstat: u8,
    /// `COMERRST` (`0x21`) — Primary-to-secondary / I2C communication error status.
    pub comerrst: u8,
    /// `CHIPSTAT` (`0x22`) — internal chip state machine status.
    pub chipstat: u8,
    /// `EVTSTICK` (`0x23`) — sticky event summary register.
    pub evtstick: u8,
    /// `UV1FCNT` (`0x24`) — primary supply undervoltage fault counter.
    pub uv1fcnt: u8,
    /// `UV2FCNT` (`0x25`) — secondary supply undervoltage fault counter.
    pub uv2fcnt: u8,
    /// `D2ECNT` (`0x26`) — chip-to-chip communication error counter.
    pub d2ecnt: u8,
    /// `ADCMVDIF` (`0x27`) — differential voltage in Volts (`raw / 6.6`).
    pub adcmvdif: f32,
    /// `ADCMGND2` (`0x28`) — secondary ground shift / negative supply in Volts (`raw / 6.6`).
    pub adcmgnd2: f32,
    /// `ADCMVCC2` (`0x29`) — secondary positive supply in Volts (`raw / 6.6`).
    pub adcmvcc2: f32,
    /// `ADCMTEMP` (`0x2A`) — gate driver die temperature in °C (`raw * 3.22 - 275.0`).
    pub adcmtemp: f32,
    /// Gate driver PCB temperature in 0.1 °C units (`0x48` I2C sensor).
    pub board_temp_c_x10: i16,
    /// Monotonic telemetry sequence number.
    pub seq: u32,
}

impl GateDriverSwitchTelemetry {
    /// Decode a 16-byte contiguous register read starting at `REG_RDYSTAT` (`0x1B..=0x2A`)
    /// using the scaling formulas from `bldc-custom/eicedriver/eicedriver.c`.
    #[must_use]
    pub fn from_status_bytes(
        switch_id: GateDriverSwitch,
        rx: &[u8; 16],
        flt_n: bool,
        rdyc: bool,
        board_temp_c_x10: i16,
        seq: u32,
    ) -> Self {
        let rdystat = rx[0];
        Self {
            switch_id,
            configured: true,
            i2c_ok: true,
            flt_n,
            rdyc,
            chip_rdy: (rdystat & 0x01) != 0,
            pri_rdy: ((rdystat >> 1) & 0x01) != 0,
            sec_rdy: ((rdystat >> 2) & 0x01) != 0,
            // rx[1] is 0x1C (unused gap between RDYSTAT and SECUVEVT)
            secuvevt: rx[2],
            gfltevt: rx[3],
            fltevt: rx[4],
            pinstat: rx[5],
            comerrst: rx[6],
            chipstat: rx[7],
            evtstick: rx[8],
            uv1fcnt: rx[9],
            uv2fcnt: rx[10],
            d2ecnt: rx[11],
            adcmvdif: f32::from(rx[12]) / 6.6,
            adcmgnd2: f32::from(rx[13]) / 6.6,
            adcmvcc2: f32::from(rx[14]) / 6.6,
            adcmtemp: f32::from(rx[15]) * 3.22 - 275.0,
            board_temp_c_x10,
            seq,
        }
    }

    /// True when the switch is configured, I2C is healthy, all three ready bits
    /// are asserted, and no fault pin or fault event register is active.
    #[must_use]
    pub const fn is_ready(&self) -> bool {
        self.configured
            && self.i2c_ok
            && self.flt_n
            && self.rdyc
            && self.chip_rdy
            && self.pri_rdy
            && self.sec_rdy
            && self.fltevt == 0
            && self.gfltevt == 0
            && self.secuvevt == 0
    }

    /// True when a hardware fault pin is asserted or any fault/error event register is non-zero.
    #[must_use]
    pub const fn has_fault(&self) -> bool {
        !self.configured
            || !self.i2c_ok
            || !self.flt_n
            || !self.rdyc
            || !self.chip_rdy
            || !self.pri_rdy
            || !self.sec_rdy
            || self.fltevt != 0
            || self.gfltevt != 0
            || self.secuvevt != 0
            || self.comerrst != 0
    }
}

/// Aggregated snapshot of all 6 EiceDRIVER gate driver switches (`UH`..`WL`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateDriverSnapshot {
    /// Per-switch telemetry indexed by [`GateDriverSwitch::index`].
    pub switches: [GateDriverSwitchTelemetry; GateDriverSwitch::COUNT],
    /// Bitmask of switches that have received at least one telemetry frame (`1 << index`).
    pub populated_mask: u8,
    /// Latest gate driver PCB temperature in 0.1 °C units.
    pub board_temp_c_x10: i16,
    /// Latest sequence number received.
    pub seq: u32,
}

impl Default for GateDriverSnapshot {
    fn default() -> Self {
        Self {
            switches: [
                GateDriverSwitchTelemetry {
                    switch_id: GateDriverSwitch::Uh,
                    ..GateDriverSwitchTelemetry::default()
                },
                GateDriverSwitchTelemetry {
                    switch_id: GateDriverSwitch::Ul,
                    ..GateDriverSwitchTelemetry::default()
                },
                GateDriverSwitchTelemetry {
                    switch_id: GateDriverSwitch::Vh,
                    ..GateDriverSwitchTelemetry::default()
                },
                GateDriverSwitchTelemetry {
                    switch_id: GateDriverSwitch::Vl,
                    ..GateDriverSwitchTelemetry::default()
                },
                GateDriverSwitchTelemetry {
                    switch_id: GateDriverSwitch::Wh,
                    ..GateDriverSwitchTelemetry::default()
                },
                GateDriverSwitchTelemetry {
                    switch_id: GateDriverSwitch::Wl,
                    ..GateDriverSwitchTelemetry::default()
                },
            ],
            populated_mask: 0,
            board_temp_c_x10: 0,
            seq: 0,
        }
    }
}

impl GateDriverSnapshot {
    /// Update the snapshot with a single switch's telemetry frame.
    pub fn update(&mut self, telem: GateDriverSwitchTelemetry) {
        let idx = telem.switch_id.index();
        self.switches[idx] = telem;
        self.populated_mask |= 1u8 << idx;
        self.board_temp_c_x10 = telem.board_temp_c_x10;
        self.seq = telem.seq;
    }

    /// Number of switches currently reporting `is_ready()`.
    #[must_use]
    pub fn ready_count(&self) -> usize {
        self.switches.iter().filter(|s| s.is_ready()).count()
    }

    /// True if any populated switch reports a fault.
    #[must_use]
    pub fn any_fault(&self) -> bool {
        self.switches
            .iter()
            .enumerate()
            .any(|(idx, s)| (self.populated_mask & (1u8 << idx)) != 0 && s.has_fault())
    }

    /// Maximum die temperature (°C) across all populated switches with valid I2C reads.
    #[must_use]
    pub fn max_die_temp_c(&self) -> Option<f32> {
        let mut max_temp: Option<f32> = None;
        for (idx, s) in self.switches.iter().enumerate() {
            if (self.populated_mask & (1u8 << idx)) != 0 && s.i2c_ok && s.adcmtemp.is_finite() {
                max_temp = Some(match max_temp {
                    Some(prev) if prev >= s.adcmtemp => prev,
                    _ => s.adcmtemp,
                });
            }
        }
        max_temp
    }
}

/// Response to a phase-source change request.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhaseSourceAck {
    /// Whether the command was enqueued to the control ISR. Confirm the
    /// actual switch via `SlowTelemetry::phase_source`.
    pub enqueued: bool,
}

// TODO: EnergyTelemetry — add when energy tracking is implemented
// Will include: amp_hours, amp_hours_charged, watt_hours, watt_hours_charged,
// tachometer (electrical revolutions). Streamed at ~1 Hz.

/// Telemetry rate configuration
///
/// Host sends this to start/stop fast telemetry streaming.
/// Device does not stream until host explicitly requests it.
/// Slow telemetry is poll-based (host controls the rate).
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryConfig {
    /// Desired fast telemetry rate in Hz. 0 = stop streaming.
    /// Device computes the closest achievable rate from FOC frequency.
    pub fast_hz: u16,
}

/// Acknowledgment for telemetry config change
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryConfigAck {
    /// Actual fast rate in Hz after applying the divider
    pub actual_fast_hz: u16,
}

/// Application protocol version. Bump on ANY breaking ICD change — wire *shape*
/// (already enforced fail-closed by ergot's schema-hashed keys) OR *semantics*
/// (units/meaning of a field whose type is unchanged, which the key hash cannot
/// see). The device reports it in [`HardwareInfo::proto_version`]; the host
/// requires an exact match before exposing the connection. The bootstrap
/// response shape itself is frozen so a future semantic version mismatch still
/// reaches this field instead of merely changing the schema-hashed endpoint
/// key. See `docs/notes/protocol-versioning.md`.
pub const ICD_PROTO_VERSION: u16 = 4;

/// Stable marker at the front of every [`HardwareInfo`] bootstrap response.
pub const ICD_BOOTSTRAP_MAGIC: [u8; 4] = *b"OXIC";

/// Hardware information returned on initial handshake
#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct HardwareInfo {
    /// Stable bootstrap marker ([`ICD_BOOTSTRAP_MAGIC`]).
    pub bootstrap_magic: [u8; 4],
    /// Application protocol version ([`ICD_PROTO_VERSION`]) — host requires an
    /// exact match.
    pub proto_version: u16,
    /// Feature bits. New optional protocol behavior must consume a bit here,
    /// not change this bootstrap struct's schema.
    pub capabilities: u32,
    /// Reserved bootstrap extension bytes. Keep the array length fixed.
    pub reserved: [u8; 8],
    /// Hardware identifier (e.g., "B-G431B-ESC1")
    pub hw: String<32>,
    /// Software version (e.g., "oxifoc-0.1.0")
    pub sw: String<32>,
    /// MCU chip name (e.g., "STM32G431CB")
    pub mcu: String<32>,
    /// 96-bit unique device ID (STM32 UID, hex-encoded)
    pub uuid: String<32>,
    /// FOC loop frequency in Hz (e.g., 20000)
    pub foc_freq_hz: u32,
    /// Hardware peak current limit in Amps
    pub max_current_a: f32,
    /// Static current-sense / vbus constants for host-side telemetry
    /// enrichment (raw frame → engineering units). See [`BoardCalib`].
    pub calib: BoardCalib,
}

impl Default for HardwareInfo {
    fn default() -> Self {
        Self {
            bootstrap_magic: ICD_BOOTSTRAP_MAGIC,
            proto_version: ICD_PROTO_VERSION,
            capabilities: 0,
            reserved: [0; 8],
            hw: String::new(),
            sw: String::new(),
            mcu: String::new(),
            uuid: String::new(),
            foc_freq_hz: 0,
            max_current_a: 0.0,
            calib: BoardCalib::default(),
        }
    }
}

/// Static board electrical constants the host needs to reconstruct engineering
/// units from the raw [`FastTelemetry`] frame (ADC counts → amps, raw bus →
/// volts). Compile-time per board; carried once at connect. Combined host-side
/// with the (dynamic) `dc_offsets` calibration and `pole_pairs` to build the
/// enrichment context. See [`crate::foc::telemetry`] and the design note
/// `docs/notes/telemetry-enrichment.md`.
///
/// This is the wire projection of the current-sense/vbus fields of
/// [`crate::foc::config::BoardConfig`] (its literal `calib` sub-struct field —
/// one definition, no bridging copy); it is intentionally NOT the whole
/// `BoardConfig` (fault thresholds, phase-sense, etc. are firmware-internal).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardCalib {
    /// Shunt resistance in Ohms.
    pub shunt_ohms: f32,
    /// Current amplifier gain (V/V).
    pub amp_gain: f32,
    /// ADC reference voltage in millivolts.
    pub adc_vref_mv: u32,
    /// Maximum ADC count (e.g. 4095 for 12-bit).
    pub adc_max_counts: u16,
    /// Current-sense sign inversion (low-side shunts, MCSDK convention).
    pub invert_current_sign: bool,
    /// VBUS divider ratio (Vbus = Vsense · ratio). Not needed to decode the
    /// frame's `vbus` (already post-divider mV) — carried for completeness.
    pub vbus_divider_ratio: f32,
}

// ============================================================================
// Fault Protocol Types
// ============================================================================

/// Maximum number of faults that can be returned in a response
pub const MAX_FAULT_RESPONSE: usize = 8;

/// Fault management request.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultRequest {
    /// Query all active faults
    #[default]
    Query,
    /// Clear against an exact observed registry generation. The request ID
    /// makes a lost-response retry unable to clear a later re-occurrence.
    Clear(Keyed<FaultClear>),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultClearTarget {
    Category(FaultCategory),
    All,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultClear {
    pub expected_generation: u32,
    pub target: FaultClearTarget,
}

/// A complete fault-registry snapshot. Topics carry snapshots rather than
/// deltas, and `generation` lets a consumer prove whether it missed one.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultSnapshot {
    /// Increments on every actual add, refinement, or clear.
    pub generation: u32,
    /// List of active faults with details (at most `MAX_FAULT_RESPONSE`)
    pub faults: Vec<FaultInfo, MAX_FAULT_RESPONSE>,
    /// Total active faults in the registry; `total > faults.len()` means
    /// the list above is truncated.
    pub total: u8,
}

/// Fault endpoint response. A conflict carries the current snapshot so the
/// host can refresh its UI without a second round trip, but must not report
/// the clear as successful.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultResponse {
    Snapshot(FaultSnapshot),
    Cleared {
        req_id: ReqId,
        snapshot: FaultSnapshot,
    },
    Conflict(FaultSnapshot),
}

// ============================================================================
// Motor Detection Protocol Types
// ============================================================================

/// Motor detection request from host.
///
/// Motor detection request — one step per request.
///
/// Each step is independent. GUI provides all required parameters explicitly
/// (e.g., resistance from a previous measurement or from saved config).
/// PI gains are computed on the host side from R and L (like VESC Tool).
// PartialEq: the device-side dedup cache verifies the request payload, not
// just the ReqId — see runtime/detect.rs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectRequest {
    /// Measure phase-to-neutral resistance.
    MeasureResistance {
        /// Max power dissipation during test (W). Controls safe test current.
        max_power_loss_w: f32,
    },
    /// Measure d/q-axis inductance via HFI.
    MeasureInductance {
        /// Max power dissipation during test (W).
        max_power_loss_w: f32,
        /// Previously measured resistance (Ω). GUI provides this.
        resistance_ohm: f32,
    },
    /// Measure flux linkage via open-loop spin.
    MeasureFlux {
        /// Max power dissipation during test (W).
        max_power_loss_w: f32,
        /// Previously measured resistance (Ω). GUI provides this.
        resistance_ohm: f32,
        /// Previously measured average inductance (H), 0.0 = unknown.
        /// Trims the ωL·i reactance term of the back-EMF-vector method.
        inductance_h: f32,
        /// Number of pole pairs.
        pole_pairs: u8,
        /// Open-loop ERPM for spin-up.
        openloop_erpm: f32,
    },
    /// Calibrate Hall sensors by sweeping electrical angle.
    /// Needs a measured resistance so the sweep current can be derived
    /// from the power class like every other step (the old parameterless
    /// variant hardcoded 2 A — too weak to drag a heavy outrunner rotor
    /// through the sweep, found on the Flipsky 5065, 2026-07-08).
    CalibrateHall {
        /// Max power dissipation during the sweep (W).
        max_power_loss_w: f32,
        /// Previously measured resistance (Ω). GUI/CLI provides this.
        resistance_ohm: f32,
    },
    /// Measure raw current-sensor zero offsets without applying them.
    MeasureCurrentOffsets {
        method: CurrentOffsetMethod,
        /// Fresh ADC samples accumulated per reported phase.
        samples_per_channel: u16,
        /// Required acknowledgement for the all-phases-at-50% topology.
        stationary_confirmed: bool,
    },
}

/// Motor detection response — matches the request step.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectResponse {
    /// Resistance measurement result.
    Resistance {
        /// Phase-to-neutral resistance (Ohms)
        resistance_ohm: f32,
    },
    /// Inductance measurement result.
    Inductance {
        /// d-axis inductance (Henries)
        inductance_d_h: f32,
        /// q-axis inductance (Henries)
        inductance_q_h: f32,
    },
    /// Flux linkage measurement result.
    FluxLinkage {
        /// Flux linkage (Weber)
        flux_linkage_wb: f32,
        /// Motor Kv (RPM/V)
        kv_rpm_per_v: f32,
    },
    /// Hall calibration succeeded.
    /// Read calibration angles via config endpoint (HallCalibration group).
    HallCalibrated,
    /// Raw current-sensor offset measurement.
    CurrentOffsets(CurrentOffsetReport),
    /// Step failed.
    Error(DetectError),
}

/// Wire-friendly detection error.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectError {
    /// Motor not responding (open circuit, disconnected)
    MotorNotResponding,
    /// Measured value outside physical range
    OutOfRange,
    /// Detection timed out
    Timeout,
    /// Hardware fault during measurement
    HardwareFault,
    /// Not enough valid samples collected
    InsufficientSamples,
    /// Measurement too noisy
    LowConfidence,
    /// Prerequisite measurement not done (e.g., inductance without resistance)
    MissingPrerequisite,
    /// Another actuator/flash operation owns the device.
    Busy,
    /// The requested topology is unsafe under the observed conditions.
    Unsafe,
}

// ============================================================================
// Configuration Protocol Types (require storage feature)
// ============================================================================

#[cfg(feature = "storage")]
pub use config_types::*;

#[cfg(feature = "storage")]
mod config_types {
    use crate::storage::{
        CurrentLimitsConfig, DcOffsetsConfig, DeratingConfigStored, FailsafeConfigStored,
        HallCalibrationConfig, HallTuningConfig, MotorParamsConfig, PiGainsConfig, PwmConfigStored,
        VelocityConfigStored, VoltageLimitsConfig,
    };

    use super::*;

    /// Configuration request from host.
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigRequest {
        /// Read a live config group with its revision and persistence state.
        Read(ConfigGroupId),
        /// Apply a value to RAM and the live driver. No flash access.
        Apply(Keyed<ConfigApply>),
        /// Persist the current live value of one group.
        Persist(Keyed<ConfigPersist>),
        /// Reset all config to defaults (erase flash)
        ResetAll,
    }

    /// Config group identifier for read requests
    #[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigGroupId {
        MotorParams,
        HallCalibration,
        DcOffsets,
        CurrentLimits,
        VoltageLimits,
        PwmConfig,
        PiGains,
        HallTuning,
        Failsafe,
        Velocity,
        // postcard encodes the variant index — append only.
        Derating,
    }

    impl ConfigGroupId {
        pub const COUNT: usize = 11;
        pub const ALL: [Self; Self::COUNT] = [
            Self::MotorParams,
            Self::HallCalibration,
            Self::DcOffsets,
            Self::CurrentLimits,
            Self::VoltageLimits,
            Self::PwmConfig,
            Self::PiGains,
            Self::HallTuning,
            Self::Failsafe,
            Self::Velocity,
            Self::Derating,
        ];

        pub const fn index(self) -> usize {
            match self {
                Self::MotorParams => 0,
                Self::HallCalibration => 1,
                Self::DcOffsets => 2,
                Self::CurrentLimits => 3,
                Self::VoltageLimits => 4,
                Self::PwmConfig => 5,
                Self::PiGains => 6,
                Self::HallTuning => 7,
                Self::Failsafe => 8,
                Self::Velocity => 9,
                Self::Derating => 10,
            }
        }
    }

    /// Config write payload — one variant per group
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigWrite {
        MotorParams(MotorParamsConfig),
        CurrentLimits(CurrentLimitsConfig),
        VoltageLimits(VoltageLimitsConfig),
        PwmConfig(PwmConfigStored),
        PiGains(PiGainsConfig),
        HallTuning(HallTuningConfig),
        /// Hall calibration result (written by the host after CalibrateHall)
        HallCalibration(HallCalibrationConfig),
        /// Current-sensor DC offsets
        DcOffsets(DcOffsetsConfig),
        /// Command-staleness deadman + failsafe policy
        Failsafe(FailsafeConfigStored),
        /// Cruise velocity-loop tuning
        Velocity(VelocityConfigStored),
        // postcard encodes the variant index — append only.
        /// Graduated derating ramps (thermal/voltage/speed)
        Derating(DeratingConfigStored),
    }

    impl ConfigWrite {
        pub const fn group(&self) -> ConfigGroupId {
            match self {
                Self::MotorParams(_) => ConfigGroupId::MotorParams,
                Self::CurrentLimits(_) => ConfigGroupId::CurrentLimits,
                Self::VoltageLimits(_) => ConfigGroupId::VoltageLimits,
                Self::PwmConfig(_) => ConfigGroupId::PwmConfig,
                Self::PiGains(_) => ConfigGroupId::PiGains,
                Self::HallTuning(_) => ConfigGroupId::HallTuning,
                Self::HallCalibration(_) => ConfigGroupId::HallCalibration,
                Self::DcOffsets(_) => ConfigGroupId::DcOffsets,
                Self::Failsafe(_) => ConfigGroupId::Failsafe,
                Self::Velocity(_) => ConfigGroupId::Velocity,
                Self::Derating(_) => ConfigGroupId::Derating,
            }
        }
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ConfigApply {
        pub expected_revision: u32,
        pub write: ConfigWrite,
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ConfigPersist {
        pub group: ConfigGroupId,
        pub expected_revision: u32,
    }

    /// Typed value returned by a revisioned read.
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigValue {
        MotorParams(MotorParamsConfig),
        CurrentLimits(CurrentLimitsConfig),
        VoltageLimits(VoltageLimitsConfig),
        PwmConfig(PwmConfigStored),
        PiGains(PiGainsConfig),
        HallTuning(HallTuningConfig),
        HallCalibration(HallCalibrationConfig),
        DcOffsets(DcOffsetsConfig),
        Failsafe(FailsafeConfigStored),
        Velocity(VelocityConfigStored),
        Derating(DeratingConfigStored),
    }

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ConfigSnapshot {
        pub group: ConfigGroupId,
        pub revision: u32,
        /// True only when flash contains this exact live revision.
        pub persisted: bool,
        pub value: Option<ConfigValue>,
    }

    /// Configuration response
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigResponse {
        Snapshot(ConfigSnapshot),
        Applied {
            req_id: ReqId,
            revision: u32,
        },
        Persisted {
            req_id: ReqId,
            revision: u32,
        },
        Reset,
        /// Flash operation failed
        Error,
        /// Refused: motor is running. Flash writes stall the chip (sector
        /// erase takes up to seconds on F405) and would glitch the FOC loop.
        Busy,
        // postcard encodes the variant index — append new variants below,
        // never reorder the ones above.
        /// Refused: the written value fails boundary validation (e.g. the
        /// current-limits headroom rule, `CurrentLimitsConfig::is_coherent`).
        Invalid,
        /// The expected revision is stale; re-read before retrying.
        Conflict {
            current_revision: u32,
        },
        /// Persistence is unavailable on this baked-config firmware profile.
        Unsupported,
    }
}

// ============================================================================
// Conversion helpers
// ============================================================================

impl MotorState {
    /// Convert from u8 (for atomic storage)
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => Self::Stopped,
            1 => Self::Running,
            _ => Self::Error,
        }
    }

    /// Convert to u8 (for atomic storage)
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Stopped => 0,
            Self::Running => 1,
            Self::Error => 2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_state_conversion() {
        assert_eq!(MotorState::from_u8(0), MotorState::Stopped);
        assert_eq!(MotorState::from_u8(1), MotorState::Running);
        assert_eq!(MotorState::from_u8(2), MotorState::Error);
        assert_eq!(MotorState::from_u8(255), MotorState::Error);

        assert_eq!(MotorState::Stopped.to_u8(), 0);
        assert_eq!(MotorState::Running.to_u8(), 1);
        assert_eq!(MotorState::Error.to_u8(), 2);
    }

    fn motor_request(session: u64, seq: u32, command: MotorCommand) -> MotorRequest {
        MotorRequest {
            source_session: session,
            seq,
            command,
        }
    }

    #[test]
    fn active_drive_is_source_owned_and_sequence_checked() {
        let active = |session, seq| {
            motor_request(
                session,
                seq,
                MotorCommand::SetMode(ControlMode::CurrentControl {
                    iq_target: 1.0,
                    id_target: 0.0,
                }),
            )
        };
        let mut ownership = DriveOwnership::new();
        ownership.record_applied(active(7, 10));

        assert_eq!(ownership.authorize(active(7, 11)), Ok(()));
        assert_eq!(
            ownership.authorize(active(7, 10)),
            Err(MotorCommandOutcome::AlreadyApplied)
        );
        assert_eq!(
            ownership.authorize(active(7, 9)),
            Err(MotorCommandOutcome::RejectedStale)
        );
        assert_eq!(
            ownership.authorize(active(8, 1)),
            Err(MotorCommandOutcome::RejectedSourceBusy)
        );
    }

    #[test]
    fn safe_and_emergency_commands_ignore_and_release_drive_owner() {
        let active = motor_request(
            7,
            u32::MAX,
            MotorCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            }),
        );
        let mut ownership = DriveOwnership::new();
        ownership.record_applied(active);

        for command in [
            MotorCommand::SetMode(ControlMode::Stopped),
            MotorCommand::SetMode(ControlMode::Coast),
            MotorCommand::SetMode(ControlMode::Brake),
            MotorCommand::EmergencyStop,
        ] {
            let safe = motor_request(99, 0, command);
            assert_eq!(ownership.authorize(safe), Ok(()));
            let mut released = ownership;
            released.record_applied(safe);
            assert_eq!(
                released.authorize(motor_request(
                    8,
                    0,
                    MotorCommand::SetMode(ControlMode::CurrentControl {
                        iq_target: 1.0,
                        id_target: 0.0,
                    })
                )),
                Ok(())
            );
        }

        assert_eq!(
            ownership.authorize(motor_request(7, 0, active.command)),
            Ok(())
        );
        assert_eq!(
            ownership.authorize(motor_request(7, u32::MAX - 1, active.command)),
            Err(MotorCommandOutcome::RejectedStale)
        );
        assert_eq!(
            ownership.authorize(motor_request(7, (1 << 31) - 1, active.command)),
            Err(MotorCommandOutcome::RejectedStale)
        );
    }

    /// Raw-Pod batch: push → postcard wire → decode must reproduce the exact
    /// samples, and the wire size must be value-independent (the whole point
    /// of raw-Pod vs per-field varints).
    #[cfg(feature = "pod")]
    #[test]
    fn batch_roundtrip() {
        let samples: Vec<FastTelemetry, FAST_BATCH_SAMPLES> = (0..FAST_BATCH_SAMPLES)
            .map(|i| FastTelemetry {
                ia: 2500 + i as u16,
                ib: 40_000,
                ic: 3,
                vbus: 6000,
                angle: (i as u16).wrapping_mul(2048),
                vd: -1200,
                vq: 32000,
                rpm: -15000,
                seq: 65_500 + i as u16, // wraps
            })
            .collect();

        let mut batch = FastTelemetryBatch::new();
        for s in &samples {
            assert!(batch.push(s));
        }
        assert!(batch.is_full());
        assert!(!batch.push(&FastTelemetry::default()));
        assert_eq!(batch.len(), FAST_BATCH_SAMPLES);

        let mut buf = [0u8; FAST_BATCH_BYTES + 8];
        let wire = postcard::to_slice(&batch, &mut buf).unwrap();
        // varint(len=576) = 2 B + payload; the container adds nothing else.
        assert_eq!(wire.len(), 2 + FAST_BATCH_BYTES);

        let decoded: FastTelemetryBatch = postcard::from_bytes(wire).unwrap();
        let out: Vec<FastTelemetry, FAST_BATCH_SAMPLES> = decoded.samples().collect();
        assert_eq!(out, samples);
    }
}
