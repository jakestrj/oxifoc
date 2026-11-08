//! CAN communication and telemetry tasks for STM32F405.

use core::sync::atomic::Ordering;

use embassy_futures::join::join;
use embassy_stm32::can::{CanRx, CanTx};
use embassy_time::{Duration, Instant, Ticker};

use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use oxifoc_core::runtime::run_telemetry_publishers;
use oxifoc_core::state::DETECTION_ACTIVE;
use oxifoc_core::timer::EmbassyTimer;

use crate::config::PWM_CONFIG;
use crate::fault::F405Fault;
use crate::transport::{can_task_read, can_task_send};
use crate::{FAULT_REGISTRY, STATE};

/// CAN RX + TX task, modeled after `hv-bms-firmware-rs/src/tasks.rs`.
#[embassy_executor::task]
pub async fn can_comm(can_rx: CanRx<'static>, can_tx: CanTx<'static>) {
    join(can_task_read(can_rx), can_task_send(can_tx)).await;
}

/// Background telemetry publisher task (`slow_telemetry_stream`,
/// `fast_telemetry_stream`, and `fault_topic_stream`).
#[embassy_executor::task]
pub async fn telemetry_task() {
    run_telemetry_publishers::<F405Fault, EmbassyTimer>(
        &STATE,
        &FAULT_REGISTRY,
        PWM_CONFIG.pwm_freq_hz,
    )
    .await;
}

/// Periodic liveness monitor and diagnostic stats logger.
#[embassy_executor::task]
pub async fn liveness_task() {
    use oxifoc_core::runtime::streaming::{
        FAST_TELEM_PERIOD, FAST_TELEM_Q, cmd_stats as cs, fast_telem_stats as s,
    };

    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut last_cmd_instant = Instant::now();
    let mut prev_reqs: u32 = 0;
    let mut stats_ticks: u8 = 0;

    loop {
        ticker.next().await;

        let cur_reqs = cs::MOTOR_REQS.load(Ordering::Relaxed);
        if cur_reqs != prev_reqs {
            prev_reqs = cur_reqs;
            last_cmd_instant = Instant::now();
        }

        let detection_active = DETECTION_ACTIVE.load(Ordering::Relaxed);
        let timed_out = !detection_active
            && Instant::now().duration_since(last_cmd_instant)
                > Duration::from_millis(LIVENESS_TIMEOUT_MS);

        if timed_out {
            let was_active = critical_section::with(|cs| {
                let mut state = STATE.borrow(cs).borrow_mut();
                let active = state.link_active;
                if active {
                    state.set_link_inactive();
                }
                active
            });
            if was_active {
                FAST_TELEM_PERIOD.store(0, Ordering::Relaxed);
                let cons = FAST_TELEM_Q.framed_consumer();
                while let Ok(grant) = cons.read() {
                    grant.release();
                }
                defmt::warn!("CAN link liveness timeout — motor stopped");
            }
        }

        stats_ticks = stats_ticks.wrapping_add(1);
        if stats_ticks >= 10 {
            stats_ticks = 0;
            defmt::info!(
                "cmd/s: reqs={} drained={} stale_max_us={} | ftelem/s: push_ok={} drops={} tx_ok={} tx_fail={}",
                cs::MOTOR_REQS.swap(0, Ordering::Relaxed),
                cs::SETMODE_DRAINED.swap(0, Ordering::Relaxed),
                cs::STALENESS_MAX_US.swap(0, Ordering::Relaxed),
                s::PUSH_OK.swap(0, Ordering::Relaxed),
                s::PUSH_DROPS.swap(0, Ordering::Relaxed),
                s::BCAST_OK.swap(0, Ordering::Relaxed),
                s::BCAST_FAILS.swap(0, Ordering::Relaxed),
            );
        }
    }
}
