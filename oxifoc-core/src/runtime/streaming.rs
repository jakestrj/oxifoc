//! Telemetry streaming via lock-free bbqueue and Embassy Watch channels.
//!
//! ISR/sim writes `FastTelemetry` structs into a bbqueue at the decimated rate.
//! Async tasks drain `FAST_TELEM_Q`, poll `STATE` for `SlowTelemetry`, and monitor
//! `FaultRegistry` changes, publishing updates into `Watch` channels consumed by
//! [`crate::runtime::servers::send_messages`].

use core::cell::RefCell;
use core::mem::size_of;
use core::sync::atomic::{AtomicU32, Ordering};

use bbqueue::nicknames::Churrasco;
use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

use crate::foc::controller::FocOutput;
use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::icd::FaultStateSnapshot;
use crate::state::{MotorControlState, update_telemetry};
use crate::timer::Timer;
pub use crate::types::FAST_BATCH_SAMPLES;
use crate::types::{FastTelemetry, GateDriverSwitchTelemetry, SlowTelemetry};

/// Number of Watch consumers supported per telemetry channel.
pub const NUM_CONSUMERS: usize = 2;

/// Watch channel for periodic slow telemetry updates.
pub static SLOW_TELEMETRY_WATCH: Watch<CriticalSectionRawMutex, SlowTelemetry, NUM_CONSUMERS> =
    Watch::new();

/// Watch channel for decimated fast telemetry updates.
pub static FAST_TELEMETRY_WATCH: Watch<CriticalSectionRawMutex, FastTelemetry, NUM_CONSUMERS> =
    Watch::new();

/// Watch channel for active fault state updates.
pub static FAULT_STATE_WATCH: Watch<CriticalSectionRawMutex, FaultStateSnapshot, NUM_CONSUMERS> =
    Watch::new();

/// Watch channel for per-switch gate driver telemetry updates.
pub static GATE_DRIVER_TELEMETRY_WATCH: Watch<
    CriticalSectionRawMutex,
    GateDriverSwitchTelemetry,
    NUM_CONSUMERS,
> = Watch::new();

/// Publish a single switch's gate driver telemetry snapshot to [`GATE_DRIVER_TELEMETRY_WATCH`].
pub fn publish_gate_driver_telemetry(telem: GateDriverSwitchTelemetry) {
    GATE_DRIVER_TELEMETRY_WATCH.sender().send(telem);
}

/// Lock-free queue for ISR → async telemetry transfer.
#[cfg(feature = "std")]
pub static FAST_TELEM_Q: Churrasco<65536> = Churrasco::new();
#[cfg(not(feature = "std"))]
pub static FAST_TELEM_Q: Churrasco<2048> = Churrasco::new();

/// Decimation period: ISR writes to bbqueue every `period` FOC cycles.
/// 0 = streaming disabled. Set when host sends `TelemetryConfigRequest { fast_hz }`.
pub static FAST_TELEM_PERIOD: AtomicU32 = AtomicU32::new(0);

/// Decimation counter for the raw diagnostic stream: emit one sample every
/// `period`-th FOC cycle.
static FAST_DECIM_CTR: AtomicU32 = AtomicU32::new(0);

/// Diagnostic counters for the fast-telemetry pipeline.
pub mod fast_telem_stats {
    use core::sync::atomic::AtomicU32;
    /// ISR-side `push_fast_telemetry` successful queue commits.
    pub static PUSH_OK: AtomicU32 = AtomicU32::new(0);
    /// ISR-side `push_fast_telemetry` grant failures (FAST_TELEM_Q full).
    pub static PUSH_DROPS: AtomicU32 = AtomicU32::new(0);
    /// Stream-task frames read from the queue with the expected length.
    pub static READ_OK: AtomicU32 = AtomicU32::new(0);
    /// Stream-task frames discarded for a wrong length.
    pub static READ_BADLEN: AtomicU32 = AtomicU32::new(0);
    /// Outgoing message sends that returned an error.
    pub static BCAST_FAILS: AtomicU32 = AtomicU32::new(0);
    /// Outgoing message sends accepted by the transport.
    pub static BCAST_OK: AtomicU32 = AtomicU32::new(0);
}

/// Command-path counters (host→device direction), 1 Hz-reported by the
/// device stats task.
pub mod cmd_stats {
    use core::sync::atomic::AtomicU32;
    /// Motor command requests received by the command handler.
    pub static MOTOR_REQS: AtomicU32 = AtomicU32::new(0);
    /// `DriverCommand::SetMode` drained by the ISR (deadman stamp events).
    pub static SETMODE_DRAINED: AtomicU32 = AtomicU32::new(0);
    /// Max command staleness (µs since the last drained `SetMode`) observed
    /// while a deadman-covered mode was active, per stats window.
    pub static STALENESS_MAX_US: AtomicU32 = AtomicU32::new(0);
}

/// Two-stage decimating anti-alias filter (CIC order 2 equivalent).
pub struct CicDecimator2<const N: usize> {
    m: u32,
    count: u32,
    a: [f32; N],
    u: [f32; N],
    a_prev: [f32; N],
    u_prev: [f32; N],
    primed: bool,
}

impl<const N: usize> CicDecimator2<N> {
    /// Unconfigured decimator (`m = 0`): `push` returns `None` until
    /// [`configure`](Self::configure) is called.
    pub const fn new() -> Self {
        Self {
            m: 0,
            count: 0,
            a: [0.0; N],
            u: [0.0; N],
            a_prev: [0.0; N],
            u_prev: [0.0; N],
            primed: false,
        }
    }

    /// Current decimation factor (0 = unconfigured).
    pub fn m(&self) -> u32 {
        self.m
    }

    /// Set the decimation factor and reset all accumulator state.
    pub fn configure(&mut self, m: u32) {
        *self = Self::new();
        self.m = m;
    }

    /// Push one input frame; returns the filtered output frame on every
    /// `m`-th call.
    pub fn push(&mut self, x: &[f32; N]) -> Option<[f32; N]> {
        if self.m == 0 {
            return None;
        }
        let w = (self.count + 1) as f32;
        for (i, &xi) in x.iter().enumerate() {
            self.a[i] += xi;
            self.u[i] += w * xi;
        }
        self.count += 1;
        if self.count < self.m {
            return None;
        }

        let mf = self.m as f32;
        let inv = 1.0 / (mf * mf);
        let mut y = [0.0f32; N];
        for (i, yi) in y.iter_mut().enumerate() {
            let desc_cur = (mf + 1.0) * self.a[i] - self.u[i];
            let asc_prev = self.u_prev[i] - self.a_prev[i];
            *yi = (asc_prev + desc_cur) * inv;
        }

        self.a_prev = self.a;
        self.u_prev = self.u;
        self.a = [0.0; N];
        self.u = [0.0; N];
        self.count = 0;

        if !self.primed && self.m > 1 {
            self.primed = true;
            return None;
        }
        self.primed = true;
        Some(y)
    }
}

impl<const N: usize> Default for CicDecimator2<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Publish one cycle's telemetry, shared by every platform ISR: update the
/// global state (waking calibration/detection listeners) and, when fast
/// streaming is enabled, emit one raw diagnostic frame every `period`-th
/// FOC cycle into the bbqueue.
pub fn publish_cycle_telemetry(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    adc: AdcSnapshot,
    hall: Option<HallSnapshot>,
    foc: FocOutput,
    pole_pairs: u8,
    seq: u32,
) {
    update_telemetry(state_mutex, adc, hall, foc);

    let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);
    if period == 0 {
        return;
    }
    if FAST_DECIM_CTR
        .fetch_add(1, Ordering::Relaxed)
        .is_multiple_of(period)
    {
        let telem = build_fast_telemetry(&adc, &foc, pole_pairs, seq);
        push_fast_telemetry(&telem);
    }
}

/// Build the compact raw diagnostic [`FastTelemetry`] from the cycle's snapshots.
pub fn build_fast_telemetry(
    adc: &AdcSnapshot,
    foc: &FocOutput,
    pole_pairs: u8,
    seq: u32,
) -> FastTelemetry {
    use core::f32::consts::TAU;
    let mech_rpm = if pole_pairs > 0 {
        foc.velocity_rad_s / f32::from(pole_pairs) * (60.0 / TAU)
    } else {
        0.0
    };
    FastTelemetry {
        ia: adc.ia,
        ib: adc.ib,
        ic: adc.ic,
        vbus: FastTelemetry::pack_vbus(adc.vbus_mv as f32 / 1000.0),
        angle: FastTelemetry::pack_angle(foc.angle_rad),
        vd: FastTelemetry::pack_volt(foc.vd),
        vq: FastTelemetry::pack_volt(foc.vq),
        rpm: FastTelemetry::pack_rpm(mech_rpm),
        seq: seq as u16,
    }
}

/// Build a [`SlowTelemetry`] snapshot from global state and fault registry.
pub fn sample_slow_telemetry<F: PlatformFault>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &FaultRegistry<F>,
    seq: u32,
) -> SlowTelemetry {
    let (
        vbus_mv,
        fet_temp,
        motor_temp,
        board_temp,
        motor_state,
        control_mode,
        phase_source,
        derating,
    ) = critical_section::with(|cs| {
        let state = state_mutex.borrow(cs).borrow();
        (
            state.last_adc.vbus_mv,
            state.last_adc.fet_temp_c_x10().unwrap_or(0),
            state.last_adc.motor_temp_c_x10().unwrap_or(0),
            state.last_adc.board_temp_c_x10().unwrap_or(0),
            state.motor_state,
            state.control_mode,
            state.phase_source,
            state.derating,
        )
    });
    SlowTelemetry {
        vbus_mv,
        fet_temp_c_x10: fet_temp,
        motor_temp_c_x10: motor_temp,
        board_temp_c_x10: board_temp,
        motor_state,
        control_mode,
        fault_count: fault_registry.count() as u8,
        phase_source,
        seq,
        derate_drive_pct: (derating.drive * 100.0) as u8,
        derate_brake_pct: (derating.brake * 100.0) as u8,
        fault_generation: fault_registry.generation(),
        derate_regen_pct: (derating.regen * 100.0) as u8,
    }
}

/// Push a `FastTelemetry` sample into the bbqueue.
pub fn push_fast_telemetry(telem: &FastTelemetry) {
    let prod = FAST_TELEM_Q.framed_producer();
    if let Ok(mut grant) = prod.grant(size_of::<FastTelemetry>() as u16) {
        grant.copy_from_slice(bytemuck::bytes_of(telem));
        grant.commit(size_of::<FastTelemetry>() as u16);
        fast_telem_stats::PUSH_OK.fetch_add(1, Ordering::Relaxed);
    } else {
        fast_telem_stats::PUSH_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Publish fault registry snapshots to [`FAULT_STATE_WATCH`] whenever faults change.
pub async fn fault_topic_stream<F: PlatformFault>(fault_registry: &'static FaultRegistry<F>) {
    let sender = FAULT_STATE_WATCH.sender();
    loop {
        let snap = fault_registry.snapshot_response();
        sender.send(FaultStateSnapshot::from_snapshot(&snap));
        fault_registry.wait_for_change().await;
    }
}

/// Periodically sample [`SlowTelemetry`] and publish it to [`SLOW_TELEMETRY_WATCH`].
pub async fn slow_telemetry_stream<F: PlatformFault, T: Timer>(
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    interval_ms: u64,
) {
    let sender = SLOW_TELEMETRY_WATCH.sender();
    let mut seq: u32 = 0;
    loop {
        seq = seq.wrapping_add(1);
        let slow = sample_slow_telemetry(state_mutex, fault_registry, seq);
        sender.send(slow);
        T::after_millis(interval_ms).await;
    }
}

/// Drain `FAST_TELEM_Q` and publish [`FastTelemetry`] samples to [`FAST_TELEMETRY_WATCH`].
pub async fn fast_telemetry_stream<T: Timer>(foc_freq_hz: u32) {
    let cons = FAST_TELEM_Q.framed_consumer();
    let sender = FAST_TELEMETRY_WATCH.sender();

    loop {
        let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);
        if period == 0 {
            T::after_millis(10).await;
            continue;
        }

        let sample_hz = foc_freq_hz / period;
        let interval_us = if sample_hz > 0 {
            (1_000_000 / u64::from(sample_hz)).max(1_000)
        } else {
            100_000
        };

        T::after_micros(interval_us).await;

        while let Ok(grant) = cons.read() {
            if grant.len() == size_of::<FastTelemetry>() {
                let sample: FastTelemetry = bytemuck::pod_read_unaligned(&grant);
                fast_telem_stats::READ_OK.fetch_add(1, Ordering::Relaxed);
                sender.send(sample);
            } else {
                fast_telem_stats::READ_BADLEN.fetch_add(1, Ordering::Relaxed);
            }
            grant.release();
        }
    }
}

#[cfg(test)]
mod cic_tests {
    use super::CicDecimator2;

    fn run(m: u32, n_out: usize, f: impl Fn(usize) -> f32) -> Vec<f32> {
        let mut cic = CicDecimator2::<1>::new();
        cic.configure(m);
        let mut out = Vec::new();
        let mut n = 0usize;
        while out.len() < n_out {
            if let Some(y) = cic.push(&[f(n)]) {
                out.push(y[0]);
            }
            n += 1;
        }
        out
    }

    #[test]
    fn dc_passes_exactly() {
        for m in [1u32, 2, 4, 8, 200] {
            let out = run(m, 5, |_| 3.25);
            for y in out {
                assert!((y - 3.25).abs() < 1e-5, "m={m}: DC distorted: {y}");
            }
        }
    }

    #[test]
    fn m1_is_identity() {
        let mut cic = CicDecimator2::<1>::new();
        cic.configure(1);
        for n in 0..10 {
            let x = (n as f32).sin();
            assert_eq!(cic.push(&[x]), Some([x]), "m=1 must be transparent");
        }
    }

    #[test]
    fn alias_band_is_nulled() {
        use core::f32::consts::TAU;
        let m = 4u32;
        let out = run(m, 16, |n| (TAU * n as f32 / m as f32 + 0.7).sin());
        let peak = out[4..].iter().fold(0.0f32, |p, y| p.max(y.abs()));
        assert!(
            peak < 0.02,
            "alias at f_out must be ≥34 dB down, got peak {peak}"
        );
    }

    #[test]
    fn reconfigure_resets() {
        let mut cic = CicDecimator2::<1>::new();
        cic.configure(2);
        cic.push(&[100.0]);
        cic.configure(2);
        let mut last = None;
        for _ in 0..6 {
            if let Some(y) = cic.push(&[1.0]) {
                last = Some(y[0]);
            }
        }
        assert!(
            (last.unwrap() - 1.0).abs() < 1e-6,
            "stale state bled through"
        );
    }
}
