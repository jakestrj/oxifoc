pub mod config;
mod connection;
pub mod ops;
pub mod transport;

use connection::CanFrameLink;
#[cfg(test)]
use connection::protocol_is_compatible;

use anyhow::Result;
use crossbeam_channel::{Receiver, Sender};
use femtopb::{EnumValue, UnknownFields};
use oxifoc_core::delivery::RetryPolicy;
use oxifoc_core::foc::phase::PhaseSource;
use oxifoc_core::icd::{
    FaultStateSnapshot, LIVENESS_TIMEOUT_MS, NMEA_TX_ARBITRATION_ID, TelemetryConfig,
};
use oxifoc_core::proto::oxifoc::{
    FaultCategory as ProtoFaultCategory, FaultClearRequest, TelemetryConfigRequest, WireMessage,
    wire_message::Payload,
};
use oxifoc_core::runtime::servers::{CommsError, NmeaCanReceiver};
use oxifoc_core::storage::{
    CurrentLimitsConfig, DcOffsetsConfig, DeratingConfigStored, FailsafeConfigStored,
    HallCalibrationConfig, HallTuningConfig, MotorParamsConfig, PiGainsConfig, PwmConfigStored,
    RuntimeConfig, VelocityConfigStored, VoltageLimitsConfig,
};
use oxifoc_core::types::{
    BoardCalib, ConfigApply, ConfigGroupId, ConfigPersist, ConfigResponse, ConfigSnapshot,
    ConfigValue, ConfigWrite, ControlMode, DetectError, DetectRequest, DetectResponse,
    FastTelemetry, FaultClearTarget, FaultRequest, FaultResponse, FaultSnapshot, HardwareInfo,
    ICD_BOOTSTRAP_MAGIC, ICD_PROTO_VERSION, Keyed, MotorCommand, MotorCommandOutcome, MotorRequest,
    MotorState, MotorStatus, SlowTelemetry, TelemetryConfigAck,
};
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU64, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use tokio::runtime::Runtime;
use tokio_util::sync::CancellationToken;
use tracing::{debug, error, info, warn};

pub use config::{HostConfig, ReconnectPolicy};
pub use transport::{TransportConfig, TransportType};

/// Logical address of the resolved motor controller.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DeviceAddress {
    pub network_id: u16,
    pub node_id: u8,
    pub port_id: u8,
}

// ── Constants ────────────────────────────────────────────────────────────────

const LINK_LOCAL_PEER: DeviceAddress = DeviceAddress {
    network_id: 0,
    node_id: 1,
    port_id: 0,
};

const RECONNECT_DELAY: Duration = Duration::from_secs(2);
/// Cadence at which the active drive setpoint is re-affirmed to the device.
const AFFIRM_INTERVAL: Duration = Duration::from_millis(50);
/// Keepalive cadence when no drive setpoint is active, keeping the device's
/// `LIVENESS_TIMEOUT_MS` (3000 ms) refreshed and ensuring multicast peers
/// stay registered.
const KEEPALIVE_INTERVAL: Duration = Duration::from_millis(500);

/// Default board electrical constants used for telemetry enrichment on virtual
/// or CAN devices that do not transmit a dedicated `HardwareInfo` frame.
const DEFAULT_CAN_CALIB: BoardCalib = BoardCalib {
    shunt_ohms: 0.003,
    amp_gain: 64.0 / 7.0,
    adc_vref_mv: 3300,
    adc_max_counts: 4095,
    invert_current_sign: true,
    vbus_divider_ratio: 10.39,
};

/// Effectively-once budget for motor detection: one ~60 s attempt, plus room
/// for a single retry whose (cached) response returns near-instantly.
pub const DETECT_POLICY: RetryPolicy = RetryPolicy {
    deadline_ms: 70_000,
    base_backoff_ms: 500,
    max_backoff_ms: 2_000,
    attempt_timeout_ms: 60_000,
};

/// Record a failed connection generation and decide whether another one is
/// permitted.
fn may_retry(policy: ReconnectPolicy, failures: &mut u32) -> bool {
    *failures = failures.saturating_add(1);
    match policy {
        ReconnectPolicy::None => false,
        ReconnectPolicy::Limited(max) => *failures < max,
        ReconnectPolicy::Infinite => true,
    }
}

fn new_drive_session() -> u64 {
    static CTR: AtomicU64 = AtomicU64::new(0);
    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;
    seed ^ CTR.fetch_add(1, Ordering::Relaxed).rotate_left(29)
}

fn next_motor_request(session: u64, seq: &mut u32, command: MotorCommand) -> MotorRequest {
    let request = MotorRequest {
        source_session: session,
        seq: *seq,
        command,
    };
    *seq = seq.wrapping_add(1);
    request
}

// ── Public API ───────────────────────────────────────────────────────────────

pub fn init_tracing() {
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .with_target(true)
        .with_level(true)
        .compact()
        .try_init();
}

/// Type alias for config response oneshot channels
pub type ConfigResponseSender = tokio::sync::oneshot::Sender<Result<ConfigResponse>>;
pub type ConfigResponseReceiver = tokio::sync::oneshot::Receiver<Result<ConfigResponse>>;

/// Create a oneshot channel pair for config request/response
pub fn config_channel() -> (ConfigResponseSender, ConfigResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Type alias for detect response oneshot channels
pub type DetectResponseSender = tokio::sync::oneshot::Sender<Result<DetectResponse>>;
pub type DetectResponseReceiver = tokio::sync::oneshot::Receiver<Result<DetectResponse>>;

/// Create a oneshot channel pair for detect request/response
pub fn detect_channel() -> (DetectResponseSender, DetectResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Acknowledged motor-command channel: carries the device's MotorStatus
/// (or the delivery error) back to the caller.
pub type MotorResponseSender = tokio::sync::oneshot::Sender<Result<MotorStatus>>;
pub type MotorResponseReceiver = tokio::sync::oneshot::Receiver<Result<MotorStatus>>;

/// Create a oneshot channel pair for an acknowledged motor command
pub fn motor_channel() -> (MotorResponseSender, MotorResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Type alias for fault query/clear response oneshot channels
pub type FaultResponseSender = tokio::sync::oneshot::Sender<Result<FaultResponse>>;
pub type FaultResponseReceiver = tokio::sync::oneshot::Receiver<Result<FaultResponse>>;

/// Create a oneshot channel pair for a fault request/response
pub fn fault_channel() -> (FaultResponseSender, FaultResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Telemetry-config ack channel.
pub type TelemetryAckSender = tokio::sync::oneshot::Sender<Result<TelemetryConfigAck>>;
pub type TelemetryAckReceiver = tokio::sync::oneshot::Receiver<Result<TelemetryConfigAck>>;

/// Create a oneshot channel pair for an acknowledged telemetry-config change
pub fn telemetry_channel() -> (TelemetryAckSender, TelemetryAckReceiver) {
    tokio::sync::oneshot::channel()
}

pub enum HostCommand {
    Motor(ControlMode),
    /// Like [`Motor`](Self::Motor) but replies with the device's status
    /// (or the delivery error).
    MotorAck(ControlMode, MotorResponseSender),
    /// Immediate high-Z stop through the motor endpoint.
    EmergencyStop(MotorResponseSender),
    SetPhaseSource(PhaseSource),
    /// Change the fast-telemetry rate.
    SetTelemetryConfig(TelemetryConfig, Option<TelemetryAckSender>),
    ConfigRead(ConfigGroupId, ConfigResponseSender),
    ConfigApply(Keyed<ConfigApply>, ConfigResponseSender),
    ConfigPersist(Keyed<ConfigPersist>, ConfigResponseSender),
    /// Erase every stored config group (factory reset).
    ConfigResetAll(ConfigResponseSender),
    Detect(DetectRequest, DetectResponseSender),
    /// Query or clear device faults.
    Fault(FaultRequest, FaultResponseSender),
}

/// Sender half of the host→backend command channel.
pub type CommandSender = tokio::sync::mpsc::UnboundedSender<HostCommand>;

pub struct HostRuntime {
    pub fast_rx: Receiver<FastTelemetry>,
    pub slow_rx: Receiver<SlowTelemetry>,
    pub fault_rx: Receiver<FaultSnapshot>,
    pub device_info_rx: Receiver<HardwareInfo>,
    pub cmd_tx: tokio::sync::mpsc::UnboundedSender<HostCommand>,
    pub connected: Arc<AtomicBool>,
    pub fast_hz: Arc<AtomicU16>,
    device_addr: Arc<AtomicU32>,
    cancel_token: CancellationToken,
    backend_thread: std::sync::Mutex<Option<thread::JoinHandle<()>>>,
}

impl HostRuntime {
    pub fn device_address(&self) -> DeviceAddress {
        unpack_addr(self.device_addr.load(Ordering::Relaxed))
    }

    pub fn wait_for_connection(&self, timeout: Duration) -> bool {
        if self.connected.load(Ordering::Relaxed) {
            return true;
        }
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.connected.load(Ordering::Relaxed) {
                return true;
            }
            thread::sleep(Duration::from_millis(50));
        }
        self.connected.load(Ordering::Relaxed)
    }

    pub fn shutdown(&self) {
        info!("Shutting down host backend...");
        self.cancel_token.cancel();
        self.join_backend(Duration::from_secs(3));
    }

    fn join_backend(&self, timeout: Duration) {
        let handle = self
            .backend_thread
            .lock()
            .expect("backend thread slot poisoned")
            .take();
        let Some(h) = handle else { return };
        let deadline = Instant::now() + timeout;
        while !h.is_finished() && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(10));
        }
        if h.is_finished() {
            let _ = h.join();
        } else {
            warn!("backend thread did not finish within {timeout:?}; exiting anyway");
        }
    }
}

pub fn build_enrich_ctx(
    cmd: &CommandSender,
    hw: Option<&HardwareInfo>,
) -> Option<oxifoc_core::foc::telemetry::EnrichCtx> {
    use oxifoc_core::types::{ConfigGroupId, ConfigValue};
    let calib = hw?.calib;
    let offsets = ops::config::read_group(cmd, ConfigGroupId::DcOffsets)
        .ok()
        .and_then(|snapshot| match snapshot.value {
            Some(ConfigValue::DcOffsets(c)) => Some((c.phase_a, c.phase_b, c.phase_c)),
            _ => None,
        })
        .unwrap_or_else(|| {
            let mid = f32::from(calib.adc_max_counts) / 2.0;
            warn!(
                "enrich: DcOffsets group unavailable — falling back to mid-scale ({mid}); \
                 reconstructed currents may carry a large DC offset"
            );
            (mid, mid, mid)
        });
    let pole_pairs = ops::config::read_group(cmd, ConfigGroupId::MotorParams)
        .ok()
        .and_then(|snapshot| match snapshot.value {
            Some(ConfigValue::MotorParams(c)) => Some(c.pole_pairs),
            _ => None,
        })
        .unwrap_or_else(|| {
            warn!("enrich: MotorParams group unavailable — pole_pairs=0, erpm column will be 0");
            0
        });
    Some(oxifoc_core::foc::telemetry::EnrichCtx::new(
        &calib, offsets, pole_pairs,
    ))
}

impl Drop for HostRuntime {
    fn drop(&mut self) {
        self.cancel_token.cancel();
        self.join_backend(Duration::from_secs(3));
    }
}

// ── Backend context ──────────────────────────────────────────────────────────

struct BackendCtx {
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    fault_sink: FaultSnapshotSink,
    info_tx: Sender<HardwareInfo>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected: Arc<AtomicBool>,
    fast_hz: Arc<AtomicU16>,
    requested_fast_hz: Arc<AtomicU16>,
    device_addr: Arc<AtomicU32>,
    cancel: CancellationToken,
}

#[derive(Clone)]
struct FaultSnapshotSink {
    tx: Sender<FaultSnapshot>,
    delivered_generation: Arc<std::sync::Mutex<Option<u32>>>,
}

impl FaultSnapshotSink {
    fn new(tx: Sender<FaultSnapshot>) -> Self {
        Self {
            tx,
            delivered_generation: Arc::new(std::sync::Mutex::new(None)),
        }
    }

    fn offer(&self, snapshot: FaultSnapshot) {
        let mut delivered = self.delivered_generation.lock().unwrap();
        if delivered.is_some_and(|current| !generation_is_newer(snapshot.generation, current)) {
            return;
        }
        if self.tx.try_send(snapshot.clone()).is_ok() {
            *delivered = Some(snapshot.generation);
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    fn needs(&self, generation: u32) -> bool {
        self.delivered_generation
            .lock()
            .unwrap()
            .is_none_or(|current| generation_is_newer(generation, current))
    }

    fn reset_generation(&self) {
        *self.delivered_generation.lock().unwrap() = None;
    }
}

fn generation_is_newer(candidate: u32, current: u32) -> bool {
    let distance = candidate.wrapping_sub(current);
    distance != 0 && distance < (1 << 31)
}

// ── Entry point ──────────────────────────────────────────────────────────────

pub fn start_host(cfg: HostConfig) -> HostRuntime {
    let (fast_tx, fast_rx) = crossbeam_channel::bounded::<FastTelemetry>(4096);
    let (slow_tx, slow_rx) = crossbeam_channel::bounded::<SlowTelemetry>(64);
    let (fault_tx, fault_rx) = crossbeam_channel::bounded::<FaultSnapshot>(64);
    let (info_tx, device_info_rx) = crossbeam_channel::bounded::<HardwareInfo>(4);
    let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));
    let fast_hz = Arc::new(AtomicU16::new(0));
    let device_addr = Arc::new(AtomicU32::new(pack_addr(LINK_LOCAL_PEER)));
    let cancel_token = CancellationToken::new();

    let ctx = BackendCtx {
        fast_tx,
        slow_tx,
        fault_sink: FaultSnapshotSink::new(fault_tx),
        info_tx,
        cmd_rx,
        connected: connected.clone(),
        fast_hz: fast_hz.clone(),
        requested_fast_hz: Arc::new(AtomicU16::new(cfg.fast_hz())),
        device_addr: device_addr.clone(),
        cancel: cancel_token.clone(),
    };

    let backend_thread = thread::spawn(move || {
        let rt = Runtime::new().expect("Failed to create tokio runtime");
        if let Err(e) = rt.block_on(backend_main(cfg, ctx)) {
            error!("backend_main error: {:?}", e);
        }
    });

    HostRuntime {
        fast_rx,
        slow_rx,
        fault_rx,
        device_info_rx,
        cmd_tx,
        connected,
        fast_hz,
        device_addr,
        cancel_token,
        backend_thread: std::sync::Mutex::new(Some(backend_thread)),
    }
}

// ── Backend dispatch ─────────────────────────────────────────────────────────

fn open_can_link(transport_config: &TransportConfig) -> Result<CanFrameLink> {
    match transport_config {
        TransportConfig::Can { interface } => {
            let ep = transport::can::connect(interface)?;
            Ok(CanFrameLink::new(ep))
        }
    }
}

fn default_can_hardware_info() -> HardwareInfo {
    let mut hw = heapless::String::new();
    let _ = hw.push_str("oxifoc-can");
    let mut sw = heapless::String::new();
    let _ = sw.push_str(env!("CARGO_PKG_VERSION"));
    let mut mcu = heapless::String::new();
    let _ = mcu.push_str("CAN/NMEA");
    let mut uuid = heapless::String::new();
    let _ = uuid.push_str("0x7E5-0x7ED");
    HardwareInfo {
        bootstrap_magic: ICD_BOOTSTRAP_MAGIC,
        proto_version: ICD_PROTO_VERSION,
        capabilities: 0,
        reserved: [0; 8],
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: 20_000,
        max_current_a: 40.0,
        calib: DEFAULT_CAN_CALIB,
    }
}

struct LocalConfigStore {
    runtime: RuntimeConfig,
    revisions: [u32; ConfigGroupId::COUNT],
}

impl Default for LocalConfigStore {
    fn default() -> Self {
        Self {
            runtime: RuntimeConfig {
                motor_params: Some(MotorParamsConfig {
                    resistance_ohm: 0.5,
                    inductance_d_h: 5e-4,
                    inductance_q_h: 5e-4,
                    flux_linkage_wb: 0.01,
                    pole_pairs: 7,
                    max_current_a: 40.0,
                    max_power_loss_w: 0.0,
                    ..Default::default()
                }),
                dc_offsets: Some(DcOffsetsConfig {
                    phase_a: 2047.5,
                    phase_b: 2047.5,
                    phase_c: 2047.5,
                }),
                current_limits: Some(CurrentLimitsConfig::default()),
                voltage_limits: Some(VoltageLimitsConfig::default()),
                pwm_config: Some(PwmConfigStored::default()),
                pi_gains: Some(PiGainsConfig::default()),
                hall_tuning: Some(HallTuningConfig::default()),
                hall_calibration: Some(HallCalibrationConfig::default()),
                failsafe: Some(FailsafeConfigStored::default()),
                velocity: Some(VelocityConfigStored::default()),
                derating: Some(DeratingConfigStored::default()),
            },
            revisions: [1; ConfigGroupId::COUNT],
        }
    }
}

impl LocalConfigStore {
    fn group_index(group: ConfigGroupId) -> usize {
        ConfigGroupId::ALL
            .iter()
            .position(|&g| g == group)
            .unwrap_or(0)
    }

    fn snapshot(&self, group: ConfigGroupId) -> ConfigSnapshot {
        let idx = Self::group_index(group);
        let value = match group {
            ConfigGroupId::MotorParams => self
                .runtime
                .motor_params
                .clone()
                .map(ConfigValue::MotorParams),
            ConfigGroupId::HallCalibration => self
                .runtime
                .hall_calibration
                .clone()
                .map(ConfigValue::HallCalibration),
            ConfigGroupId::DcOffsets => self.runtime.dc_offsets.clone().map(ConfigValue::DcOffsets),
            ConfigGroupId::CurrentLimits => self
                .runtime
                .current_limits
                .clone()
                .map(ConfigValue::CurrentLimits),
            ConfigGroupId::VoltageLimits => self
                .runtime
                .voltage_limits
                .clone()
                .map(ConfigValue::VoltageLimits),
            ConfigGroupId::PwmConfig => self.runtime.pwm_config.clone().map(ConfigValue::PwmConfig),
            ConfigGroupId::PiGains => self.runtime.pi_gains.clone().map(ConfigValue::PiGains),
            ConfigGroupId::HallTuning => self
                .runtime
                .hall_tuning
                .clone()
                .map(ConfigValue::HallTuning),
            ConfigGroupId::Failsafe => self.runtime.failsafe.clone().map(ConfigValue::Failsafe),
            ConfigGroupId::Velocity => self.runtime.velocity.clone().map(ConfigValue::Velocity),
            ConfigGroupId::Derating => self.runtime.derating.clone().map(ConfigValue::Derating),
        };
        ConfigSnapshot {
            group,
            revision: self.revisions[idx],
            persisted: value.is_some(),
            value,
        }
    }

    fn apply(&mut self, req: Keyed<ConfigApply>) -> ConfigResponse {
        let group = req.inner.write.group();
        let idx = Self::group_index(group);
        if req.inner.expected_revision != self.revisions[idx] {
            return ConfigResponse::Conflict {
                current_revision: self.revisions[idx],
            };
        }
        match req.inner.write {
            ConfigWrite::MotorParams(v) => self.runtime.motor_params = Some(v),
            ConfigWrite::HallCalibration(v) => self.runtime.hall_calibration = Some(v),
            ConfigWrite::DcOffsets(v) => self.runtime.dc_offsets = Some(v),
            ConfigWrite::CurrentLimits(v) => self.runtime.current_limits = Some(v),
            ConfigWrite::VoltageLimits(v) => self.runtime.voltage_limits = Some(v),
            ConfigWrite::PwmConfig(v) => self.runtime.pwm_config = Some(v),
            ConfigWrite::PiGains(v) => self.runtime.pi_gains = Some(v),
            ConfigWrite::HallTuning(v) => self.runtime.hall_tuning = Some(v),
            ConfigWrite::Failsafe(v) => self.runtime.failsafe = Some(v),
            ConfigWrite::Velocity(v) => self.runtime.velocity = Some(v),
            ConfigWrite::Derating(v) => self.runtime.derating = Some(v),
        }
        self.revisions[idx] = self.revisions[idx].wrapping_add(1);
        ConfigResponse::Applied {
            req_id: req.id,
            revision: self.revisions[idx],
        }
    }

    fn persist(&self, req: Keyed<ConfigPersist>) -> ConfigResponse {
        let idx = Self::group_index(req.inner.group);
        if req.inner.expected_revision != self.revisions[idx] {
            return ConfigResponse::Conflict {
                current_revision: self.revisions[idx],
            };
        }
        ConfigResponse::Persisted {
            req_id: req.id,
            revision: self.revisions[idx],
        }
    }

    fn reset_all(&mut self) -> ConfigResponse {
        *self = Self::default();
        ConfigResponse::Reset
    }
}

async fn backend_main(cfg: HostConfig, mut ctx: BackendCtx) -> Result<()> {
    let transport_type = cfg.transport_type();
    info!("Oxifoc Host backend - transport: {:?}", transport_type);

    let transport_config = cfg.transport_config()?;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;
    let mut config_store = LocalConfigStore::default();
    let drive_session = new_drive_session();
    let mut motor_seq = 1u32;

    loop {
        if ctx.cancel.is_cancelled() {
            break;
        }
        let link_res = open_can_link(&transport_config);

        let mut link = match link_res {
            Ok(l) => l,
            Err(e) => {
                ctx.connected.store(false, Ordering::Relaxed);
                let retry = may_retry(policy, &mut connect_attempts);
                warn!(
                    "Transport open failed (attempt {}): {:?}",
                    connect_attempts, e
                );
                if !retry {
                    info!("Reconnect policy exhausted — giving up");
                    break;
                }
                tokio::select! {
                    _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                    _ = ctx.cancel.cancelled() => break,
                }
            }
        };

        // Send initial TelemetryConfigRequest on 0x7E5 to wake the link and
        // register our peer address on the virtual CAN bus.
        let initial_fast_hz = ctx.requested_fast_hz.load(Ordering::Relaxed);
        let init_msg = WireMessage {
            payload: Some(Payload::TelemetryConfig(TelemetryConfigRequest {
                fast_hz: u32::from(initial_fast_hz),
                unknown_fields: UnknownFields::empty(),
            })),
            unknown_fields: UnknownFields::empty(),
        };
        if let Err(e) = link.send_wire_message(&init_msg).await {
            warn!("Initial telemetry config send failed: {e:#}");
        } else {
            ctx.fast_hz.store(initial_fast_hz, Ordering::Relaxed);
        }

        let mut nmea_rx = NmeaCanReceiver::new();
        let mut active_setpoint: Option<ControlMode> = None;
        let mut last_slow: Option<SlowTelemetry> = None;
        let mut latest_fault_snap = FaultSnapshot::default();
        let mut last_rx_at = Instant::now();
        let mut affirm_ticker = tokio::time::interval(AFFIRM_INTERVAL);
        affirm_ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        let mut keepalive_ticker = tokio::time::interval(KEEPALIVE_INTERVAL);
        keepalive_ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        let session_lost = loop {
            tokio::select! {
                _ = ctx.cancel.cancelled() => break false,

                frame_res = link.recv_frame() => {
                    match frame_res {
                        Ok((arb_id, data)) => {
                            if arb_id != NMEA_TX_ARBITRATION_ID {
                                continue;
                            }
                            last_rx_at = Instant::now();
                            if !ctx.connected.load(Ordering::Relaxed) {
                                ctx.connected.store(true, Ordering::Relaxed);
                                ctx.device_addr
                                    .store(u32::from(NMEA_TX_ARBITRATION_ID), Ordering::Relaxed);
                                ctx.fault_sink.reset_generation();
                                connect_attempts = 0;
                                let _ = ctx.info_tx.try_send(default_can_hardware_info());
                                info!("CAN device link active (RX=0x7ED, TX=0x7E5)");
                            }
                            let decoded = nmea_rx.push_frame_with(&data, |proto| {
                                match proto.payload.as_ref() {
                                    Some(Payload::SlowTelemetry(s)) => {
                                        Ok(DecodedCanMessage::Slow(SlowTelemetry::from(s)))
                                    }
                                    Some(Payload::FastTelemetry(f)) => {
                                        Ok(DecodedCanMessage::Fast(FastTelemetry::from(f)))
                                    }
                                    Some(Payload::FaultState(flt)) => {
                                        Ok(DecodedCanMessage::Fault(FaultStateSnapshot::from(flt)))
                                    }
                                    _ => Err(CommsError::InvalidRequest),
                                }
                            });
                            match decoded {
                                Ok(Some(DecodedCanMessage::Slow(slow))) => {
                                    last_slow = Some(slow);
                                    let _ = ctx.slow_tx.try_send(slow);
                                }
                                Ok(Some(DecodedCanMessage::Fast(fast))) => {
                                    let _ = ctx.fast_tx.try_send(fast);
                                }
                                Ok(Some(DecodedCanMessage::Fault(fault_state))) => {
                                    let snap = fault_state.to_fault_snapshot();
                                    latest_fault_snap = snap.clone();
                                    ctx.fault_sink.offer(snap);
                                }
                                Ok(None) => {}
                                Err(e) => {
                                    debug!("CAN NMEA decode error: {e:?}");
                                }
                            }
                        }
                        Err(e) => {
                            warn!("CAN link read error: {e:#}");
                            break true;
                        }
                    }
                }

                cmd = ctx.cmd_rx.recv() => {
                    let Some(cmd) = cmd else { break false };
                    if !ctx.connected.load(Ordering::Relaxed) {
                        active_setpoint = None;
                        reject_unverified_command(cmd);
                        continue;
                    }
                    handle_can_command(
                        &mut link,
                        cmd,
                        &ctx.fast_hz,
                        &ctx.requested_fast_hz,
                        &mut active_setpoint,
                        drive_session,
                        &mut motor_seq,
                        last_slow.as_ref(),
                        &mut latest_fault_snap,
                        &mut config_store,
                    ).await;
                }

                _ = affirm_ticker.tick() => {
                    if ctx.connected.load(Ordering::Relaxed)
                        && last_rx_at.elapsed() > Duration::from_millis(LIVENESS_TIMEOUT_MS)
                    {
                        warn!("No CAN frames received within {LIVENESS_TIMEOUT_MS}ms; marking link inactive");
                        ctx.connected.store(false, Ordering::Relaxed);
                        active_setpoint = None;
                    }
                    if let Some(mode) = active_setpoint {
                        let req = next_motor_request(
                            drive_session,
                            &mut motor_seq,
                            MotorCommand::SetMode(mode),
                        );
                        let msg = WireMessage {
                            payload: Some(Payload::MotorCommand(req.into())),
                            unknown_fields: UnknownFields::empty(),
                        };
                        if let Err(e) = link.send_wire_message(&msg).await {
                            warn!("Setpoint affirm send failed: {e:#}");
                        }
                    }
                }

                _ = keepalive_ticker.tick() => {
                    if active_setpoint.is_none() {
                        let hz = ctx.requested_fast_hz.load(Ordering::Relaxed);
                        let msg = WireMessage {
                            payload: Some(Payload::TelemetryConfig(TelemetryConfigRequest {
                                fast_hz: u32::from(hz),
                                unknown_fields: UnknownFields::empty(),
                            })),
                            unknown_fields: UnknownFields::empty(),
                        };
                        let _ = link.send_wire_message(&msg).await;
                    }
                }
            }
        };

        ctx.connected.store(false, Ordering::Relaxed);
        if !session_lost {
            break;
        }
        if !may_retry(policy, &mut connect_attempts) {
            info!("Reconnect policy exhausted after connection loss");
            break;
        }
        tokio::select! {
            _ = tokio::time::sleep(RECONNECT_DELAY) => {}
            _ = ctx.cancel.cancelled() => break,
        }
        info!("Attempting reconnection...");
    }

    ctx.connected.store(false, Ordering::Relaxed);
    info!("Host backend shutdown complete");
    Ok(())
}

enum DecodedCanMessage {
    Slow(SlowTelemetry),
    Fast(FastTelemetry),
    Fault(FaultStateSnapshot),
}

#[expect(
    clippy::too_many_arguments,
    reason = "flat backend dispatch; a struct would just move the names"
)]
async fn handle_can_command(
    link: &mut CanFrameLink,
    cmd: HostCommand,
    fast_hz_flag: &Arc<AtomicU16>,
    requested_fast_hz: &Arc<AtomicU16>,
    active_setpoint: &mut Option<ControlMode>,
    drive_session: u64,
    motor_seq: &mut u32,
    last_slow: Option<&SlowTelemetry>,
    latest_fault_snap: &mut FaultSnapshot,
    config_store: &mut LocalConfigStore,
) {
    let current_status = |mode_override: Option<ControlMode>| -> MotorStatus {
        let state = match mode_override.or_else(|| last_slow.map(|s| s.control_mode)) {
            Some(ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake) | None => {
                last_slow.map_or(MotorState::Stopped, |s| s.motor_state)
            }
            Some(_) => MotorState::Running,
        };
        MotorStatus {
            state,
            mode: mode_override
                .unwrap_or_else(|| last_slow.map_or(ControlMode::Stopped, |s| s.control_mode)),
            fault_count: last_slow.map_or(latest_fault_snap.total, |s| s.fault_count),
            outcome: MotorCommandOutcome::Applied,
        }
    };

    match cmd {
        HostCommand::Motor(mc) => {
            *active_setpoint = if matches!(
                mc,
                ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
            ) {
                None
            } else {
                Some(mc)
            };
            let request = next_motor_request(drive_session, motor_seq, MotorCommand::SetMode(mc));
            let msg = WireMessage {
                payload: Some(Payload::MotorCommand(request.into())),
                unknown_fields: UnknownFields::empty(),
            };
            if let Err(e) = link.send_wire_message(&msg).await {
                warn!("Motor command send failed: {e:#}");
            }
        }
        HostCommand::MotorAck(mc, reply_tx) => {
            *active_setpoint = if matches!(
                mc,
                ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
            ) {
                None
            } else {
                Some(mc)
            };
            let request = next_motor_request(drive_session, motor_seq, MotorCommand::SetMode(mc));
            let msg = WireMessage {
                payload: Some(Payload::MotorCommand(request.into())),
                unknown_fields: UnknownFields::empty(),
            };
            let res = link
                .send_wire_message(&msg)
                .await
                .map(|()| current_status(Some(mc)));
            let _ = reply_tx.send(res);
        }
        HostCommand::EmergencyStop(reply_tx) => {
            *active_setpoint = None;
            let request = next_motor_request(drive_session, motor_seq, MotorCommand::EmergencyStop);
            let msg = WireMessage {
                payload: Some(Payload::MotorCommand(request.into())),
                unknown_fields: UnknownFields::empty(),
            };
            let res = link
                .send_wire_message(&msg)
                .await
                .map(|()| current_status(Some(ControlMode::Stopped)));
            let _ = reply_tx.send(res);
        }
        HostCommand::SetPhaseSource(source) => {
            let msg = WireMessage {
                payload: Some(Payload::PhaseSourceRequest(source.to_proto_request())),
                unknown_fields: UnknownFields::empty(),
            };
            if let Err(e) = link.send_wire_message(&msg).await {
                warn!("Phase source command failed: {e:#}");
            }
        }
        HostCommand::SetTelemetryConfig(cfg, reply) => {
            requested_fast_hz.store(cfg.fast_hz, Ordering::Relaxed);
            let msg = WireMessage {
                payload: Some(Payload::TelemetryConfig(TelemetryConfigRequest {
                    fast_hz: u32::from(cfg.fast_hz),
                    unknown_fields: UnknownFields::empty(),
                })),
                unknown_fields: UnknownFields::empty(),
            };
            match link.send_wire_message(&msg).await {
                Ok(()) => {
                    fast_hz_flag.store(cfg.fast_hz, Ordering::Relaxed);
                    if let Some(reply) = reply {
                        let _ = reply.send(Ok(TelemetryConfigAck {
                            actual_fast_hz: cfg.fast_hz,
                        }));
                    }
                }
                Err(e) => {
                    warn!("Telemetry config send failed: {e:#}");
                    if let Some(reply) = reply {
                        let _ = reply.send(Err(e));
                    }
                }
            }
        }
        HostCommand::Fault(req, reply_tx) => match req {
            FaultRequest::Query => {
                let _ = reply_tx.send(Ok(FaultResponse::Snapshot(latest_fault_snap.clone())));
            }
            FaultRequest::Clear(keyed) => {
                let (clear_all, category) = match keyed.inner.target {
                    FaultClearTarget::All => (true, ProtoFaultCategory::None),
                    FaultClearTarget::Category(cat) => (false, cat.into()),
                };
                let msg = WireMessage {
                    payload: Some(Payload::FaultClear(FaultClearRequest {
                        expected_generation: keyed.inner.expected_generation,
                        clear_all,
                        category: EnumValue::Known(category),
                        unknown_fields: UnknownFields::empty(),
                    })),
                    unknown_fields: UnknownFields::empty(),
                };
                match link.send_wire_message(&msg).await {
                    Ok(()) => {
                        *latest_fault_snap = FaultSnapshot {
                            generation: latest_fault_snap.generation.wrapping_add(1),
                            faults: heapless::Vec::new(),
                            total: 0,
                        };
                        let _ = reply_tx.send(Ok(FaultResponse::Cleared {
                            req_id: keyed.id,
                            snapshot: latest_fault_snap.clone(),
                        }));
                    }
                    Err(e) => {
                        let _ = reply_tx.send(Err(e));
                    }
                }
            }
        },
        HostCommand::ConfigRead(group_id, reply_tx) => {
            let _ = reply_tx.send(Ok(ConfigResponse::Snapshot(
                config_store.snapshot(group_id),
            )));
        }
        HostCommand::ConfigApply(request, reply_tx) => {
            let _ = reply_tx.send(Ok(config_store.apply(request)));
        }
        HostCommand::ConfigPersist(request, reply_tx) => {
            let _ = reply_tx.send(Ok(config_store.persist(request)));
        }
        HostCommand::ConfigResetAll(reply_tx) => {
            let _ = reply_tx.send(Ok(config_store.reset_all()));
        }
        HostCommand::Detect(_req, reply_tx) => {
            let _ = reply_tx.send(Ok(DetectResponse::Error(DetectError::MotorNotResponding)));
        }
    }
}

fn pack_addr(a: DeviceAddress) -> u32 {
    (u32::from(a.network_id) << 16) | (u32::from(a.node_id) << 8) | u32::from(a.port_id)
}

fn unpack_addr(v: u32) -> DeviceAddress {
    DeviceAddress {
        network_id: (v >> 16) as u16,
        node_id: (v >> 8) as u8,
        port_id: v as u8,
    }
}

fn reject_unverified_command(cmd: HostCommand) {
    let error = || anyhow::anyhow!("device is not connected and identity-verified");
    match cmd {
        HostCommand::Motor(mode) => {
            tracing::warn!("Dropping motor command before verified handshake: {mode:?}");
        }
        HostCommand::MotorAck(_, reply) | HostCommand::EmergencyStop(reply) => {
            let _ = reply.send(Err(error()));
        }
        HostCommand::SetPhaseSource(source) => {
            tracing::warn!("Dropping phase-source command before verified handshake: {source:?}");
        }
        HostCommand::SetTelemetryConfig(config, reply) => {
            tracing::warn!("Dropping telemetry config before verified handshake: {config:?}");
            if let Some(reply) = reply {
                let _ = reply.send(Err(error()));
            }
        }
        HostCommand::ConfigRead(_, reply)
        | HostCommand::ConfigApply(_, reply)
        | HostCommand::ConfigPersist(_, reply)
        | HostCommand::ConfigResetAll(reply) => {
            let _ = reply.send(Err(error()));
        }
        HostCommand::Detect(_, reply) => {
            let _ = reply.send(Err(error()));
        }
        HostCommand::Fault(_, reply) => {
            let _ = reply.send(Err(error()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        FaultSnapshotSink, generation_is_newer, may_retry, protocol_is_compatible,
        reject_unverified_command,
    };
    use crate::ReconnectPolicy;
    use oxifoc_core::types::FaultSnapshot;

    #[test]
    fn protocol_version_must_match_exactly() {
        let current = oxifoc_core::types::ICD_PROTO_VERSION;
        assert!(protocol_is_compatible(current));
        assert!(!protocol_is_compatible(current.wrapping_add(1)));
        assert!(!protocol_is_compatible(current.wrapping_sub(1)));
    }

    #[test]
    fn reconnect_policy_covers_every_connection_generation() {
        let mut failures = 0;
        assert!(!may_retry(ReconnectPolicy::None, &mut failures));
        assert_eq!(failures, 1);

        let mut failures = 0;
        assert!(may_retry(ReconnectPolicy::Limited(3), &mut failures));
        assert!(may_retry(ReconnectPolicy::Limited(3), &mut failures));
        assert!(!may_retry(ReconnectPolicy::Limited(3), &mut failures));
        assert_eq!(failures, 3);

        let mut failures = u32::MAX;
        assert!(may_retry(ReconnectPolicy::Infinite, &mut failures));
        assert_eq!(failures, u32::MAX);
    }

    #[test]
    fn fault_generation_order_handles_wrap() {
        assert!(generation_is_newer(2, 1));
        assert!(!generation_is_newer(1, 1));
        assert!(!generation_is_newer(1, 2));
        assert!(generation_is_newer(0, u32::MAX));
        assert!(!generation_is_newer(u32::MAX, 0));
    }

    #[test]
    fn full_fault_consumer_queue_keeps_generation_dirty() {
        let (tx, rx) = crossbeam_channel::bounded(1);
        let sink = FaultSnapshotSink::new(tx);
        let snapshot = |generation| FaultSnapshot {
            generation,
            ..Default::default()
        };

        sink.offer(snapshot(1));
        sink.offer(snapshot(2)); // full: must not claim generation 2 delivered
        assert!(sink.needs(2));
        assert_eq!(rx.recv().unwrap().generation, 1);

        sink.offer(snapshot(2));
        sink.offer(snapshot(1)); // stale: must not regress the consumer view
        assert_eq!(rx.recv().unwrap().generation, 2);
        assert!(!sink.needs(2));
    }

    #[test]
    fn fault_generation_resets_across_verified_reconnect() {
        let (tx, rx) = crossbeam_channel::bounded(2);
        let sink = FaultSnapshotSink::new(tx);
        sink.offer(FaultSnapshot {
            generation: 100,
            ..Default::default()
        });
        assert_eq!(rx.recv().unwrap().generation, 100);

        sink.reset_generation();
        sink.offer(FaultSnapshot {
            generation: 0,
            ..Default::default()
        });
        assert_eq!(rx.recv().unwrap().generation, 0);
    }

    #[test]
    fn unverified_connection_rejects_acknowledged_commands() {
        let (tx, rx) = crate::motor_channel();
        reject_unverified_command(super::HostCommand::EmergencyStop(tx));
        let error = rx
            .blocking_recv()
            .expect("rejection sender must answer")
            .expect_err("unverified command must fail");
        assert!(error.to_string().contains("identity-verified"));
    }
}
