//! Infineon EiceDRIVER (1ED38x0Mc12M family) gate-driver driver and GD PCB
//! I2C temperature monitor for the JBLDC high-voltage inverter (`board-jbldc`).
//!
//! # Hardware Architecture (`hw_jbldc.h` / `eicedriver.c`)
//!
//! Six isolated gate drivers sit on `I2C1` (`PB6` SCL / `PB7` SDA @ 100 kHz)
//! alongside a 12-bit gate-driver PCB temperature sensor at `0x48`.
//!
//! All six EiceDRIVER ICs power up at the same default I2C address (`0x0D`).
//! Before `TIM1` claims the six PWM `IN` pins (`PA8/PA9/PA10` + `PB13/PB14/PB15`),
//! [`configure_all`] assigns each switch its individual + group I2C address by:
//! 1. Holding all six PWM `IN` pins low so unselected chips ignore `0x0D`.
//! 2. Driving one switch's `RDYC` pin low, waiting 10 ms, and driving its PWM
//!    `IN` pin high to select that single chip.
//! 3. Writing `[REG_I2CADD, dev_addr, group_addr, 0x01]` to `0x0D` to program
//!    and lock its address (`0x0E..=0x13`).
//! 4. Releasing `IN` low and returning `RDYC` to floating input.
//! 5. Programming `REG_ADCCFG`, `REG_D1LVL`, `REG_D1FILT`, `REG_UVTLVL`, and
//!    locking configuration via `REG_CFGOK = 0x01` at `dev_addr`.

use core::sync::atomic::{AtomicI16, Ordering};

use embassy_stm32::gpio::{Flex, Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, Config as I2cConfig, Error as I2cError, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Ticker, Timer};
use oxifoc_core::foc::config::NtcConfig;
use oxifoc_core::foc::fault::FaultCategory;
use oxifoc_core::runtime::streaming::publish_gate_driver_telemetry;
use oxifoc_core::types::{GateDriverSwitch, GateDriverSwitchTelemetry};

use super::resources::{DrvResources, MotorResources};
use crate::FAULT_REGISTRY;
use crate::fault::{EiceDriverFaultStatus, F405Fault};

// ============================================================================
// I2C Addresses & EiceDRIVER Register Map
// ============================================================================

/// Default power-on I2C address shared by all unconfigured EiceDRIVER ICs.
const ADDR_DEFAULT: u8 = 0x0D;

/// Gate-driver PCB temperature sensor address on `I2C1` (`hw_jbldc.c`).
const ADDR_GD_TEMP_SENSOR: u8 = 0x48;

/// EiceDRIVER configuration registers (`eicedriver_registers.h`).
const REG_I2CADD: u8 = 0x00;
const REG_UVTLVL: u8 = 0x06;
const REG_ADCCFG: u8 = 0x09;
const REG_D1LVL: u8 = 0x0C;
const REG_D1FILT: u8 = 0x0D;
const REG_CFGOK: u8 = 0x1C;
const REG_CHIPSTAT: u8 = 0x2D;

/// `CHIPSTAT` bit 3 (`CONFIG`): high when input-side logic is configured (`CFGOK`).
const CHIPSTAT_CONFIG_BIT: u8 = 1 << 3;

/// `RDYSTAT` bits `[2:0]`: `CHIP_RDY | PRI_RDY | SEC_RDY` (`STATUS_READY = 0x07`).
const RDYSTAT_ALL_READY: u8 = 0x07;

/// `GFLTEVT` (`0x29`) active-low fault bits `[1:0]`: `PRI_FLTN | SEC_FLTN` (`0x03` = no fault).
const GFLTEVT_NO_FAULT: u8 = GateDriverSwitchTelemetry::GFLTEVT_NO_FAULT;

#[derive(Clone, Copy)]
struct SwitchMeta {
    name: &'static str,
    dev_addr: u8,
    group_addr: u8,
}

/// Per-switch I2C address and group-address table (`hw_jbldc.c::configure_gate_drivers`).
const SWITCHES: [SwitchMeta; 6] = [
    SwitchMeta {
        name: "UH",
        dev_addr: 0x0E,
        group_addr: 0x14,
    },
    SwitchMeta {
        name: "UL",
        dev_addr: 0x0F,
        group_addr: 0x15,
    },
    SwitchMeta {
        name: "VH",
        dev_addr: 0x10,
        group_addr: 0x14,
    },
    SwitchMeta {
        name: "VL",
        dev_addr: 0x11,
        group_addr: 0x16,
    },
    SwitchMeta {
        name: "WH",
        dev_addr: 0x12,
        group_addr: 0x15,
    },
    SwitchMeta {
        name: "WL",
        dev_addr: 0x13,
        group_addr: 0x16,
    },
];

// ============================================================================
// Configuration & Status Types
// ============================================================================

/// DESAT1 voltage threshold selection (`eicedriver.c`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[allow(dead_code)]
pub enum Desat1VoltageThresh {
    /// 1.85 V (`0x00`)
    V1_85,
    /// 4.79 V (`0x0F` — JBLDC default in `hw_jbldc.h`)
    V4_79,
    /// 5.27 V (`0x11`)
    V5_27,
    /// Disable DESAT1 (`0x20`, mapped from `DESAT1_VOLTAGE_THRESH_9V18` in `eicedriver.c`)
    Disabled,
}

impl Desat1VoltageThresh {
    const fn reg_val(self) -> u8 {
        match self {
            Self::V1_85 => 0x00,
            Self::V4_79 => 0x0F,
            Self::V5_27 => 0x11,
            Self::Disabled => 0x20,
        }
    }
}

/// DESAT1 blanking/filter time selection (`eicedriver.c`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[allow(dead_code)]
pub enum Desat1FilterTime {
    /// 75 ns (`0x01 << 1`)
    Ns75,
    /// 375 ns (`0x07 << 1` — JBLDC default in `hw_jbldc.h`)
    Ns375,
    /// 1175 ns (`0x0F << 1`)
    Ns1175,
    /// 2775 ns (`0x17 << 1`)
    Ns2775,
    /// 5975 ns (`0x1F << 1`)
    Ns5975,
}

impl Desat1FilterTime {
    const fn reg_val(self) -> u8 {
        match self {
            Self::Ns75 => 0x01 << 1,
            Self::Ns375 => 0x07 << 1,
            Self::Ns1175 => 0x0F << 1,
            Self::Ns2775 => 0x17 << 1,
            Self::Ns5975 => 0x1F << 1,
        }
    }
}

/// Programmable EiceDRIVER protection parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct EiceDriverConfig {
    pub desat1_voltage_thresh: Desat1VoltageThresh,
    pub desat1_filter_time: Desat1FilterTime,
}

impl EiceDriverConfig {
    /// Default parameters from `hw_jbldc.h` (`4.79 V`, `375 ns`).
    pub const JBLDC_DEFAULT: Self = Self {
        desat1_voltage_thresh: Desat1VoltageThresh::V4_79,
        desat1_filter_time: Desat1FilterTime::Ns375,
    };
}

/// Parsed 16-byte status block from an EiceDRIVER (`eicedriver_read_status`).
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
#[allow(dead_code)]
pub struct EiceDriverValues {
    pub rdy_raw: u8,
    pub chip_rdy: bool,
    pub pri_rdy: bool,
    pub sec_rdy: bool,
    pub secuvevt: u8,
    pub gfltevt: u8,
    pub fltevt: u8,
    pub pinstat: u8,
    pub comerrst: u8,
    pub chipstat: u8,
    pub evtstick: u8,
    pub uv1fcnt: u8,
    pub uv2fcnt: u8,
    pub d2ecnt: u8,
    pub adcmvdif_raw: u8,
    pub adcmgnd2_raw: u8,
    pub adcmvcc2_raw: u8,
    pub adcmtemp_raw: u8,
}

impl EiceDriverValues {
    fn from_status_bytes(rx: &[u8; 16]) -> Self {
        let rdy_raw = rx[0] & RDYSTAT_ALL_READY;
        Self {
            rdy_raw,
            chip_rdy: (rx[0] & (1 << 0)) != 0,
            pri_rdy: (rx[0] & (1 << 1)) != 0,
            sec_rdy: (rx[0] & (1 << 2)) != 0,
            // rx[1] is reserved
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
            adcmvdif_raw: rx[12],
            adcmgnd2_raw: rx[13],
            adcmvcc2_raw: rx[14],
            adcmtemp_raw: rx[15],
        }
    }

    #[inline]
    fn has_fault_or_unready(&self) -> bool {
        self.rdy_raw != RDYSTAT_ALL_READY
            || self.fltevt != 0
            || (self.gfltevt & GFLTEVT_NO_FAULT) != GFLTEVT_NO_FAULT
            || self.secuvevt != 0
    }
}

/// Error returned if any EiceDRIVER switch fails boot configuration.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct EiceDriverInitError {
    pub switch_idx: u8,
    pub step: &'static str,
    pub i2c_err: I2cError,
}

/// Latest gate-driver PCB temperature from `0x48` on `I2C1` in `0.1 °C` units.
///
/// Fed into `TempSensorId::Board` on every `AdcSnapshot` in `control::foc`.
pub static GD_BOARD_TEMP_C_X10: AtomicI16 = AtomicI16::new(NtcConfig::INVALID_TEMP_C_X10);

/// Owned `I2C1` bus and EiceDRIVER control/status pins handed to [`eicedriver_monitor_task`].
pub struct EiceDriverMonitor {
    i2c: I2c<'static, Blocking, Master>,
    rdyc: [Flex<'static>; 6],
    flt_n: [Input<'static>; 6],
}

// ============================================================================
// Initialization & Per-Switch Programming
// ============================================================================

fn new_rdyc_flex(
    pin: embassy_stm32::Peri<'static, impl embassy_stm32::gpio::Pin>,
) -> Flex<'static> {
    let mut flex = Flex::new(pin);
    flex.set_as_input(Pull::None);
    flex
}

/// Initialize `I2C1` + `RDYC`/`FLT_N` pins, assign addresses `0x0E..=0x13` to
/// all six EiceDRIVERs, and program their protection registers.
///
/// Must be called **before** `MotorPwm::new` so the six PWM pins in `motor`
/// can be temporarily driven as push-pull GPIO outputs during chip-select
/// address assignment.
pub async fn configure_all(
    motor: &mut MotorResources,
    drv: DrvResources,
) -> (EiceDriverMonitor, Result<(), EiceDriverInitError>) {
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000);
    i2c_cfg.gpio_speed = Speed::Medium;
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;
    i2c_cfg.timeout = Duration::from_millis(5);

    let mut i2c = I2c::new_blocking(drv.i2c1, drv.pb6, drv.pb7, i2c_cfg);

    let mut rdyc = [
        new_rdyc_flex(drv.pb4),  // UH
        new_rdyc_flex(drv.pc10), // UL
        new_rdyc_flex(drv.pc12), // VH
        new_rdyc_flex(drv.pc5),  // VL
        new_rdyc_flex(drv.pc7),  // WH
        new_rdyc_flex(drv.pa3),  // WL
    ];

    let flt_n = [
        Input::new(drv.pc11, Pull::None), // UH
        Input::new(drv.pa15, Pull::None), // UL
        Input::new(drv.pd2, Pull::None),  // VH
        Input::new(drv.pb0, Pull::None),  // VL
        Input::new(drv.pc6, Pull::None),  // WH
        Input::new(drv.pa4, Pull::None),  // WL
    ];

    // Match hw_jbldc.c::hw_init_gpio settle delay after I2C1 bring-up.
    Timer::after_millis(10).await;

    // Prime GD PCB temperature (and recover I2C1 immediately if a slave held SDA low across reset).
    if poll_gd_pcb_temp(&mut i2c).is_err() {
        defmt::warn!("JBLDC I2C1 initial 0x48 temp read failed; running bus recovery");
        try_restore_i2c(&mut i2c).await;
        let _ = poll_gd_pcb_temp(&mut i2c);
    }

    let mut first_err: Option<EiceDriverInitError> = None;
    {
        // Hold all 6 PWM IN pins LOW as push-pull outputs so unselected drivers
        // cannot respond to ADDR_DEFAULT (0x0D). Dropped at scope exit so
        // MotorPwm::new takes clean ownership of `motor`.
        let mut pwm_pins = [
            Output::new(motor.pa8.reborrow(), Level::Low, Speed::VeryHigh), // UH
            Output::new(motor.pb13.reborrow(), Level::Low, Speed::VeryHigh), // UL
            Output::new(motor.pa9.reborrow(), Level::Low, Speed::VeryHigh), // VH
            Output::new(motor.pb14.reborrow(), Level::Low, Speed::VeryHigh), // VL
            Output::new(motor.pa10.reborrow(), Level::Low, Speed::VeryHigh), // WH
            Output::new(motor.pb15.reborrow(), Level::Low, Speed::VeryHigh), // WL
        ];

        for i in 0..SWITCHES.len() {
            let sw = SWITCHES[i];
            if let Err((step, i2c_err)) = configure_single_driver(
                &mut i2c,
                &mut pwm_pins[i],
                &mut rdyc[i],
                sw,
                &EiceDriverConfig::JBLDC_DEFAULT,
            )
            .await
            {
                defmt::error!(
                    "EiceDRIVER {} ({=u8:#x}) config failed at {}: {}",
                    sw.name,
                    sw.dev_addr,
                    step,
                    i2c_err
                );
                if first_err.is_none() {
                    first_err = Some(EiceDriverInitError {
                        switch_idx: i as u8,
                        step,
                        i2c_err,
                    });
                }
            }
        }
    }

    let result = match first_err {
        None => {
            defmt::info!(
                "All 6 EiceDRIVERs configured (0x0E..=0x13, DESAT1=4.79V / 375ns, UVTLVL=0x06)"
            );
            Ok(())
        }
        Some(err) => Err(err),
    };

    (EiceDriverMonitor { i2c, rdyc, flt_n }, result)
}

async fn configure_single_driver(
    i2c: &mut I2c<'static, Blocking, Master>,
    pwm_pin: &mut Output<'_>,
    rdyc_pin: &mut Flex<'static>,
    sw: SwitchMeta,
    cfg: &EiceDriverConfig,
) -> Result<(), (&'static str, I2cError)> {
    if is_driver_configured(i2c, sw.dev_addr) {
        defmt::info!(
            "EiceDRIVER {} ({=u8:#x}) already configured",
            sw.name,
            sw.dev_addr
        );
        return Ok(());
    }

    // Enter chip-select address configuration: RDYC low -> settle -> IN high.
    rdyc_pin.set_low();
    rdyc_pin.set_as_output(Speed::VeryHigh);
    Timer::after_millis(10).await;
    pwm_pin.set_high();

    let addr_res = i2c.blocking_write(
        ADDR_DEFAULT,
        &[REG_I2CADD, sw.dev_addr, sw.group_addr, 0x01],
    );

    if addr_res.is_ok() {
        Timer::after_millis(10).await;
    }

    // Always release IN low and RDYC back to input, even if the I2C write failed.
    pwm_pin.set_low();
    rdyc_pin.set_as_input(Pull::None);

    addr_res.map_err(|e| ("REG_I2CADD@0x0D", e))?;

    // Parameter configuration state at the newly assigned individual address.
    i2c.blocking_write(sw.dev_addr, &[REG_ADCCFG, 15])
        .map_err(|e| ("REG_ADCCFG", e))?;
    i2c.blocking_write(
        sw.dev_addr,
        &[REG_D1LVL, cfg.desat1_voltage_thresh.reg_val()],
    )
    .map_err(|e| ("REG_D1LVL", e))?;
    i2c.blocking_write(sw.dev_addr, &[REG_D1FILT, cfg.desat1_filter_time.reg_val()])
        .map_err(|e| ("REG_D1FILT", e))?;
    i2c.blocking_write(sw.dev_addr, &[REG_UVTLVL, 0x06])
        .map_err(|e| ("REG_UVTLVL", e))?;
    i2c.blocking_write(sw.dev_addr, &[REG_CFGOK, 0x01])
        .map_err(|e| ("REG_CFGOK", e))?;

    defmt::info!(
        "EiceDRIVER {} configured at addr={=u8:#x} group={=u8:#x}",
        sw.name,
        sw.dev_addr,
        sw.group_addr
    );
    Ok(())
}

/// Check `CHIPSTAT.CONFIG` (bit 3) at `dev_addr` (`is_eicedriver_configured` in `eicedriver.c`).
fn is_driver_configured(i2c: &mut I2c<'static, Blocking, Master>, dev_addr: u8) -> bool {
    let mut rx = [0u8; 1];
    if i2c
        .blocking_write_read(dev_addr, &[REG_CHIPSTAT], &mut rx)
        .is_ok()
    {
        (rx[0] & CHIPSTAT_CONFIG_BIT) != 0
    } else {
        false
    }
}

/// Read all 16 status registers starting at `REG_RDYSTAT` (`eicedriver_read_status`).
fn read_driver_status_bytes(
    i2c: &mut I2c<'static, Blocking, Master>,
    dev_addr: u8,
) -> Result<[u8; 16], i2c::Error> {
    let mut rx = [0u8; 16];
    i2c.blocking_read(dev_addr, &mut rx)?;
    Ok(rx)
}

/// Read the 12-bit GD PCB temperature sensor (`0x48`, reg `0x00`) and store `0.1 °C` in
/// [`GD_BOARD_TEMP_C_X10`].
fn poll_gd_pcb_temp(i2c: &mut I2c<'static, Blocking, Master>) -> Result<i16, I2cError> {
    let mut rx = [0u8; 2];
    i2c.blocking_write_read(ADDR_GD_TEMP_SENSOR, &[0x00], &mut rx)?;
    let raw = i16::from_be_bytes(rx);
    // 0.0625 °C/LSB = (1/16) °C -> in 0.1 °C units: (counts * 10) / 16 = (counts * 5) / 8.
    let temp_c_x10 = ((i32::from(raw >> 4) * 5) / 8) as i16;
    GD_BOARD_TEMP_C_X10.store(temp_c_x10, Ordering::Relaxed);
    Ok(temp_c_x10)
}

/// Recover a wedged `I2C1` bus (`PB6` SCL / `PB7` SDA) without dropping `I2c`:
/// 16 open-drain SCL recovery clocks + START/STOP condition + `I2C1.CR1.SWRST`
/// errata sequence (`hw_jbldc.c::hw_try_restore_i2c` + STM32F4 I2C errata).
async fn try_restore_i2c(_i2c: &mut I2c<'static, Blocking, Master>) {
    let gpiob = pac::GPIOB;

    // PB6 (SCL) and PB7 (SDA) already have OTYPER=OpenDrain, PUPDR=PullUp, AFR=AF4
    // from I2c::new_blocking. Switch MODER to Output with both lines high.
    gpiob.bsrr().write(|w| {
        w.set_bs(6, true);
        w.set_bs(7, true);
    });
    critical_section::with(|_| {
        gpiob.moder().modify(|w| {
            w.set_moder(6, pac::gpio::vals::Moder::Output);
            w.set_moder(7, pac::gpio::vals::Moder::Output);
        });
    });

    Timer::after_micros(50).await;

    for _ in 0..16 {
        gpiob.bsrr().write(|w| w.set_br(6, true));
        Timer::after_micros(50).await;
        gpiob.bsrr().write(|w| w.set_bs(6, true));
        Timer::after_micros(50).await;
    }

    // Generate START (SDA low while SCL high) then STOP (SDA high while SCL high).
    gpiob.bsrr().write(|w| w.set_br(7, true));
    Timer::after_micros(50).await;
    gpiob.bsrr().write(|w| w.set_br(6, true));
    Timer::after_micros(50).await;
    gpiob.bsrr().write(|w| w.set_bs(6, true));
    Timer::after_micros(50).await;
    gpiob.bsrr().write(|w| w.set_bs(7, true));
    Timer::after_micros(50).await;

    // Return PB6/PB7 to I2C1 AF4 open-drain mode.
    critical_section::with(|_| {
        gpiob.moder().modify(|w| {
            w.set_moder(6, pac::gpio::vals::Moder::Alternate);
            w.set_moder(7, pac::gpio::vals::Moder::Alternate);
        });
    });

    // Reset I2C1 state machine via SWRST while preserving Embassy's timing registers.
    let i2c1 = pac::I2C1;
    let cr2 = i2c1.cr2().read();
    let ccr = i2c1.ccr().read();
    let trise = i2c1.trise().read();

    i2c1.cr1().modify(|w| w.set_pe(false));
    i2c1.cr1().modify(|w| w.set_swrst(true));
    i2c1.cr1().modify(|w| w.set_swrst(false));
    i2c1.cr2().write_value(cr2);
    i2c1.ccr().write_value(ccr);
    i2c1.trise().write_value(trise);
    i2c1.cr1().modify(|w| w.set_pe(true));
}

/// Pulse all six open-drain `RDYC` pins low for 1 ms to reset latched EiceDRIVER faults.
async fn reset_latched_faults(rdyc: &mut [Flex<'static>; 6]) {
    for pin in rdyc.iter_mut() {
        pin.set_low();
        pin.set_as_input_output(Speed::Low);
    }
    Timer::after_millis(1).await;
    for pin in rdyc.iter_mut() {
        pin.set_as_input(Pull::None);
    }
    // Allow isolated secondary status transfer to refresh before reading status.
    Timer::after_millis(2).await;
}

// ============================================================================
// Background Monitor Task
// ============================================================================

/// 10 Hz EiceDRIVER + GD PCB temperature monitor task (`hw_jbldc.c::temp_thread` +
/// `eicedriver_read_status`).
///
/// - Polls the `0x48` GD PCB temperature sensor into [`GD_BOARD_TEMP_C_X10`],
///   recovering `I2C1` on bus errors.
/// - Polls the 16-byte status block of all six EiceDRIVERs (`0x0E..=0x13`) and
///   raises `F405Fault::DrvFault` if any switch loses `RDYSTAT` (`0x07`) or
///   latches `FLTEVT` / `GFLTEVT` / `SECUVEVT`.
/// - Pulses `RDYC` low to clear hardware fault latches only after the host
///   explicitly clears `FaultCategory::DriverFault`.
#[embassy_executor::task]
pub async fn eicedriver_monitor_task(mut monitor: EiceDriverMonitor, gd_configured: bool) {
    defmt::info!(
        "EiceDRIVER + I2C temp monitor started (gd_configured={})",
        gd_configured
    );

    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut hw_fault_latched = false;
    let mut last_logged: Option<EiceDriverFaultStatus> = None;
    let mut rr_switch_idx: usize = 0;
    let mut telem_seq: u32 = 0;

    loop {
        ticker.next().await;

        let board_temp_c_x10 = match poll_gd_pcb_temp(&mut monitor.i2c) {
            Ok(temp) => temp,
            Err(_) => {
                GD_BOARD_TEMP_C_X10.store(NtcConfig::INVALID_TEMP_C_X10, Ordering::Relaxed);
                try_restore_i2c(&mut monitor.i2c).await;
                0
            }
        };

        if !gd_configured {
            // Boot configuration failed: keep DriverFault asserted even if the host
            // sends a clear command so unconfigured IGBT drivers can never be switched.
            FAULT_REGISTRY.set(F405Fault::DrvConfigFailed);
            let switch_id = GateDriverSwitch::ALL[rr_switch_idx];
            let flt_n = monitor.flt_n[rr_switch_idx].is_high();
            let rdyc = monitor.rdyc[rr_switch_idx].is_high();
            telem_seq = telem_seq.wrapping_add(1);
            publish_gate_driver_telemetry(GateDriverSwitchTelemetry {
                switch_id,
                configured: false,
                i2c_ok: false,
                flt_n,
                rdyc,
                board_temp_c_x10,
                seq: telem_seq,
                ..GateDriverSwitchTelemetry::default()
            });
            rr_switch_idx = (rr_switch_idx + 1) % GateDriverSwitch::COUNT;
            continue;
        }

        // Host explicitly cleared DriverFault -> pulse RDYC low to unlatch EiceDRIVERs
        // before re-polling hardware status.
        if hw_fault_latched && !FAULT_REGISTRY.has_category(FaultCategory::DriverFault) {
            defmt::info!("Host cleared DriverFault; pulsing RDYC low to reset EiceDRIVERs");
            reset_latched_faults(&mut monitor.rdyc).await;
            hw_fault_latched = false;
            last_logged = None;
        }

        let mut i2c_err_mask: u8 = 0;
        let mut unready_mask: u8 = 0;
        let mut first_fault: Option<(u8, EiceDriverValues)> = None;
        let mut first_i2c_err_sw: Option<u8> = None;
        let mut switch_telemetry = [GateDriverSwitchTelemetry::default(); GateDriverSwitch::COUNT];

        for (idx, sw) in SWITCHES.iter().enumerate() {
            let bit = 1u8 << idx;
            let switch_id = GateDriverSwitch::ALL[idx];
            let flt_n = monitor.flt_n[idx].is_high();
            let rdyc = monitor.rdyc[idx].is_high();
            telem_seq = telem_seq.wrapping_add(1);

            match read_driver_status_bytes(&mut monitor.i2c, sw.dev_addr) {
                Ok(rx) => {
                    let vals = EiceDriverValues::from_status_bytes(&rx);
                    // Fault/ready detection uses the digital I2C registers rather than
                    // raw FLT_N GPIO reads, matching hw_jbldc.c::is_gate_driver_fault()
                    // where FLT_N GPIO polling is disabled due to board pin noise.
                    if vals.rdy_raw != RDYSTAT_ALL_READY {
                        unready_mask |= bit;
                    }
                    if vals.has_fault_or_unready() && first_fault.is_none() {
                        first_fault = Some((idx as u8, vals));
                    }
                    switch_telemetry[idx] = GateDriverSwitchTelemetry::from_status_bytes(
                        switch_id,
                        &rx,
                        flt_n,
                        rdyc,
                        board_temp_c_x10,
                        telem_seq,
                    );
                }
                Err(_) => {
                    i2c_err_mask |= bit;
                    if first_i2c_err_sw.is_none() {
                        first_i2c_err_sw = Some(idx as u8);
                    }
                    switch_telemetry[idx] = GateDriverSwitchTelemetry {
                        switch_id,
                        configured: true,
                        i2c_ok: false,
                        flt_n,
                        rdyc,
                        board_temp_c_x10,
                        seq: telem_seq,
                        ..GateDriverSwitchTelemetry::default()
                    };
                }
            }
        }

        // Always advance round-robin across UH..WL so all 6 switches remain populated
        // in the host UI even if an earlier switch has an active fault.
        let publish_idx = rr_switch_idx;
        rr_switch_idx = (rr_switch_idx + 1) % GateDriverSwitch::COUNT;
        publish_gate_driver_telemetry(switch_telemetry[publish_idx]);

        if i2c_err_mask != 0 {
            try_restore_i2c(&mut monitor.i2c).await;
        }

        if first_fault.is_some() || i2c_err_mask != 0 {
            let status = if let Some((sw_idx, vals)) = first_fault {
                EiceDriverFaultStatus {
                    i2c_err_mask,
                    unready_mask,
                    first_switch: sw_idx,
                    rdy_stat: vals.rdy_raw,
                    fltevt: vals.fltevt,
                    gfltevt: vals.gfltevt,
                    secuvevt: vals.secuvevt,
                }
            } else {
                EiceDriverFaultStatus {
                    i2c_err_mask,
                    unready_mask,
                    first_switch: first_i2c_err_sw.unwrap_or(0),
                    rdy_stat: 0,
                    fltevt: 0,
                    gfltevt: GFLTEVT_NO_FAULT,
                    secuvevt: 0,
                }
            };

            if last_logged != Some(status) {
                let sw_name = SWITCHES
                    .get(usize::from(status.first_switch))
                    .map_or("GD", |s| s.name);
                defmt::error!(
                    "EiceDRIVER fault/unready: first={} rdy={=u8:#x} flt={=u8:#x} gflt={=u8:#x} \
                     uv={=u8:#x} unready_mask={=u8:#x} i2c_err_mask={=u8:#x}",
                    sw_name,
                    status.rdy_stat,
                    status.fltevt,
                    status.gfltevt,
                    status.secuvevt,
                    status.unready_mask,
                    status.i2c_err_mask,
                );
                last_logged = Some(status);
            }

            FAULT_REGISTRY.set(F405Fault::DrvFault(status));
            hw_fault_latched = true;
        } else if hw_fault_latched {
            defmt::info!("EiceDRIVER hardware status healthy across all 6 switches");
            hw_fault_latched = false;
            last_logged = None;
        }
    }
}
