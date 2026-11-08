#![no_std]
#![no_main]

#[cfg(not(any(feature = "board-cf2", feature = "board-vesc6-mk5")))]
compile_error!("select exactly one board feature: board-cf2 or board-vesc6-mk5");
#[cfg(all(feature = "board-cf2", feature = "board-vesc6-mk5"))]
compile_error!("board-cf2 and board-vesc6-mk5 are mutually exclusive");

use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::{bind_interrupts, exti, interrupt};
use embassy_time::{Duration, Timer};

// Bind EXTI9_5 interrupt for nFAULT monitoring (PB7/EXTI7)
bind_interrupts!(struct ExtiIrqs {
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
});

// Module declarations
mod calibration;
mod config;
mod control;
pub mod fault;
mod hardware;
mod motor;
mod protocol;
// Panic/HardFault handlers (gate kill) + IWDG live here.
mod safety;
mod sensors;
mod storage;
mod transport;

#[cfg(feature = "board-vesc6-mk5")]
#[allow(unused_imports)]
use hardware::BoardCtrlResources;
#[allow(unused_imports)]
use hardware::{AssignedResources, CanResources, DrvResources, HallResources, MotorResources};
use motor::MotorPwm;

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::F405Fault);

/// Global runtime config — loaded from flash at boot, read by config_server for protocol access.
pub static RUNTIME_CONFIG: critical_section::Mutex<
    core::cell::RefCell<oxifoc_core::storage::RuntimeConfig>,
> = critical_section::Mutex::new(core::cell::RefCell::new(
    oxifoc_core::storage::RuntimeConfig {
        motor_params: None,
        hall_calibration: None,
        dc_offsets: None,
        current_limits: None,
        voltage_limits: None,
        pwm_config: None,
        pi_gains: None,
        hall_tuning: None,
        failsafe: None,
        velocity: None,
        derating: None,
    },
));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    // Comms/housekeeping IRQs sit BELOW the control ISRs. The FOC ADC ISR
    // runs at 0 and the hall capture (TIM3) at 1, but every other vector
    // ships at the reset default 0: an in-flight handler at the SAME level
    // cannot be preempted, so a USB/UART/RNG/EXTI/time-driver handler
    // delayed ADC entry by its full duration. Demoting them lets the FOC
    // ISR preempt immediately.
    {
        use embassy_stm32::interrupt::{InterruptExt, Priority};
        interrupt::CAN1_TX.set_priority(Priority::P5);
        interrupt::CAN1_RX0.set_priority(Priority::P5);
        interrupt::CAN1_RX1.set_priority(Priority::P5);
        interrupt::CAN1_SCE.set_priority(Priority::P5);
        interrupt::EXTI9_5.set_priority(Priority::P4); // DRV8301 nFAULT
        interrupt::TIM2.set_priority(Priority::P6); // embassy time driver
    }

    // ========== STEP 2: Initialize defmt logging over RTT ==========
    rtt_target::rtt_init_defmt!();

    // ========== STEP 3: Initialize LEDs ==========
    let green_led = hardware::peripherals::init_green_led(p.PB0);
    let red_led = hardware::peripherals::init_red_led(p.PB1);
    spawner.spawn(defmt::unwrap!(heartbeat(green_led)));
    spawner.spawn(defmt::unwrap!(fault_led(red_led)));

    // ========== STEP 4: Split Hardware Resources ==========
    let r = split_resources!(p);

    // MK5: latch the power button (PC5) and enable the current/phase-sense
    // filters ASAP — releasing the button before PC5 goes high powers the
    // board off. The Outputs must live for the whole run.
    #[cfg(feature = "board-vesc6-mk5")]
    let _board_ctrl = hardware::board_early_init(r.board_ctrl);

    // ========== STEP 5: Initialize CAN1 Transport & Protocol Tasks ==========
    let (can_tx, can_rx) = transport::init_can(r.can).await;
    spawner.spawn(defmt::unwrap!(protocol::servers::can_comm(can_rx, can_tx)));
    spawner.spawn(defmt::unwrap!(protocol::servers::telemetry_task()));
    spawner.spawn(defmt::unwrap!(protocol::servers::liveness_task()));

    // ========== STEP 8: Initialize Persistent Storage ==========
    let flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH);
    let flash = embassy_embedded_hal::adapter::BlockingAsync::new(flash);
    spawner.spawn(defmt::unwrap!(storage::storage_worker(flash)));
    let runtime_config = storage::CONFIG_LOADED.wait().await;
    critical_section::with(|cs| RUNTIME_CONFIG.borrow(cs).replace(runtime_config.clone()));
    defmt::info!("Config loaded from flash");

    // ========== STEP 9: Initialize DRV8301 Gate Driver ==========
    let (drv_config, nfault) = hardware::drv8301::init_bus(r.drv, ExtiIrqs);

    let (drv_spi, drv_result) = hardware::drv8301::configure_drv8301(drv_config);
    match drv_result {
        Ok(()) => {
            defmt::info!("DRV8301 ready");
            hardware::drv8301::enable_gate_driver();
        }
        Err(_e) => {
            // An unconfigured DRV runs on its power-on defaults: shunt gain
            // 10 where the board calib assumes 20 (every current reads 2x
            // low, the software OC trip is effectively doubled) and no
            // VDS-OCP programmed. On the MK5's bit-bang SPI one flaky wire
            // lands exactly here. Kill-class fault + gate driver held off:
            // even after a fault clear the bridge cannot switch.
            defmt::error!("DRV8301 configuration failed: DriverFault raised, gate driver disabled");
            hardware::drv8301::disable_gate_driver();
            FAULT_REGISTRY.set(fault::F405Fault::DrvConfigFailed);
        }
    }
    spawner.spawn(defmt::unwrap!(hardware::drv8301::nfault_monitor_task(
        nfault, drv_spi
    )));

    // ========== STEP 10: Initialize Hall Sensor ==========
    sensors::init_hall(r.hall.pc6, r.hall.pc7, r.hall.pc8, p.TIM3);

    // ========== STEP 11: Initialize ADCs ==========
    let adc_handles = hardware::peripherals::init_adc(
        p.ADC1, p.ADC2, p.ADC3, p.PC0, p.PA3, p.PC1, p.PC4, p.PC2, p.PC3,
    );

    // ========== STEP 12: Initialize Motor PWM ==========
    let motor_pwm = MotorPwm::new(r.motor, config::PWM_CONFIG);

    // ========== STEP 13: Initialize FOC Controller ==========
    control::foc::init(motor_pwm, adc_handles, &runtime_config).await;

    // FOC ISR is running now — arm the watchdog it feeds.
    safety::arm_watchdog(p.IWDG);

    // 1 Hz ISR-cost stats (same isr/s line format as the g431).
    spawner.spawn(defmt::unwrap!(control::foc::isr_stats_task()));

    #[cfg(feature = "board-cf2")]
    defmt::info!(
        "F405 board=CF2: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
         SPI3 CS/SCK/MISO/MOSI=PC9/PC10/PC11/PC12, halls=PC6/7/8, ADC currents PC0-2, VBUS PC3, \
         CAN1 RX=PB8 TX=PB9"
    );
    #[cfg(feature = "board-vesc6-mk5")]
    defmt::info!(
        "F405 board=VESC6_MK5: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
         bit-bang SPI CS/SCK/MISO/MOSI=PC9/PC10/PB3/PB4, halls=PC6/7/8, ADC currents PC0-2 \
         (phase shunts), VBUS PC3, CAN1 RX=PB8 TX=PB9, latch PC5, filters PD2/PC13"
    );
    defmt::info!(
        "Board config: shunt={=f32}Ω, amp_gain={=f32} V/V, vbus_ratio={=f32}:1, faults={}",
        config::BOARD.calib.shunt_ohms,
        config::BOARD.calib.amp_gain,
        config::BOARD.calib.vbus_divider_ratio,
        FAULT_REGISTRY.count()
    );
}

// ========== Background Tasks ==========

/// Blink green LED so we know the scheduler is alive
#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(50)).await;
        led.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}

/// Red LED fault indicator — on when any fault is active
#[embassy_executor::task]
async fn fault_led(mut led: Output<'static>) {
    loop {
        if FAULT_REGISTRY.any() {
            led.set_low();
        } else {
            led.set_high();
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
