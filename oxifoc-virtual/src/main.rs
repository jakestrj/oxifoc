mod can_server;
mod sim;
mod storage;

pub struct TokioTimer;

impl Timer for TokioTimer {
    async fn after_millis(ms: u64) {
        tokio::time::sleep(std::time::Duration::from_millis(ms)).await;
    }

    async fn after_micros(us: u64) {
        tokio::time::sleep(std::time::Duration::from_micros(us)).await;
    }
}

use clap::Parser;
use oxifoc_core::storage::{CONFIG_LOADED, MotorParamsConfig};
use oxifoc_core::timer::Timer;
use oxifoc_core::virtual_motor::MotorParams;
use tracing_subscriber::EnvFilter;

// Platform state globals
oxifoc_core::define_platform_state!(oxifoc_core::foc::fault::StandardFault);

/// Synthetic board electrical constants reported to the host (for telemetry
/// enrichment) and used by the sim to encode currents into raw ADC counts —
/// generic B-G431B-ESC1-like values so the enrich round-trip is meaningful.
pub const VIRTUAL_CALIB: oxifoc_core::types::BoardCalib = oxifoc_core::types::BoardCalib {
    shunt_ohms: 0.003,
    amp_gain: 64.0 / 7.0,
    adc_vref_mv: 3300,
    adc_max_counts: 4095,
    invert_current_sign: true,
    vbus_divider_ratio: 10.39,
};

#[derive(Clone, Copy, Debug, PartialEq, clap::ValueEnum)]
enum Transport {
    #[value(alias = "vcan", alias = "socketcan")]
    Can,
}

#[derive(Parser)]
#[command(name = "oxifoc-virtual")]
#[command(about = "Virtual motor controller with streaming-protocol and NMEA over CAN")]
struct Args {
    /// Transport protocol
    #[arg(short, long, default_value = "can")]
    transport: Transport,
    /// SocketCAN / Virtual CAN interface name
    #[arg(short, long, default_value = "vcan0")]
    can_interface: String,
    /// Initial fast telemetry rate in Hz (0 = disabled until host requests)
    #[arg(long, default_value_t = 0)]
    fast_hz: u32,
    /// Simulated FOC frequency in Hz
    #[arg(short, long, default_value_t = 20_000)]
    foc_freq: u32,
    /// Simulation batch size (steps per sleep)
    #[arg(short, long, default_value_t = 100)]
    batch: usize,
    /// DC bus voltage (V)
    #[arg(long, default_value_t = 24.0)]
    vbus: f32,
    /// Motor pole pairs
    #[arg(long, default_value_t = 7)]
    pole_pairs: u8,
    /// Maximum phase current (A)
    #[arg(long, default_value_t = 40.0)]
    max_current: f32,
    /// Load torque in N·m (opposes rotation, creates steady-state speed)
    #[arg(long, default_value_t = 0.0)]
    load: f32,
}

fn config_from_motor_params(params: MotorParams, max_current_a: f32) -> MotorParamsConfig {
    MotorParamsConfig {
        resistance_ohm: params.r,
        inductance_d_h: params.ld,
        inductance_q_h: params.lq,
        flux_linkage_wb: params.lambda,
        pole_pairs: params.pole_pairs,
        max_current_a,
        max_power_loss_w: 0.0,
        ..Default::default()
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    oxifoc_core::state::BOOT_CURRENT_OFFSET_PENDING
        .store(false, core::sync::atomic::Ordering::Release);
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let args = Args::parse();
    if args.fast_hz > 0 {
        let period = (args.foc_freq / args.fast_hz.max(1)).max(1);
        oxifoc_core::runtime::streaming::FAST_TELEM_PERIOD
            .store(period, core::sync::atomic::Ordering::Relaxed);
    }
    tracing::info!(
        "oxifoc-virtual: {:?} can={}, foc={}Hz, fast={}Hz, batch={}, vbus={}V, pp={}, max_i={}A",
        args.transport,
        args.can_interface,
        args.foc_freq,
        args.fast_hz,
        args.batch,
        args.vbus,
        args.pole_pairs,
        args.max_current,
    );

    let motor_params = MotorParams {
        pole_pairs: args.pole_pairs,
        ..Default::default()
    };

    // Storage worker uses !Send futures (sequential-storage internals),
    // so run it on a dedicated thread with a LocalSet.
    std::thread::spawn(|| {
        if let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        {
            rt.block_on(storage::storage_worker());
        }
    });

    let mut loaded_config = CONFIG_LOADED.wait().await;
    if loaded_config.motor_params.is_none() {
        loaded_config.motor_params = Some(config_from_motor_params(motor_params, args.max_current));
    }
    tracing::info!("Config loaded from mock flash");

    // Spawn simulation loop
    tokio::spawn(sim::foc_loop(
        args.foc_freq,
        args.batch,
        args.vbus,
        args.load,
        motor_params,
        sim::board(args.max_current),
        loaded_config,
        &STATE,
        &FAULT_REGISTRY,
    ));

    // Run server (blocks on recv loop)
    match args.transport {
        Transport::Can => {
            can_server::run(&args.can_interface, args.foc_freq, &STATE, &FAULT_REGISTRY).await
        }
    }
}
