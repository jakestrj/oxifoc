//! Configuration constants for oxifoc-f405 (per-board `BOARD` const below)

#[cfg(not(feature = "board-jbldc"))]
use oxifoc_core::foc::config::NtcTopology;
use oxifoc_core::foc::config::{BoardConfig, NtcConfig, PhaseSense};
use oxifoc_core::foc::pwm::MotorPwmConfig;
use oxifoc_core::types::BoardCalib;

// ============================================================================
// Board Hardware Configuration
// ============================================================================

/// Cheap FOCer 2 / Simple FOCer 2 board configuration
///
/// Hardware specs:
/// - Shunt resistors: 2x 1mΩ in parallel = 0.5mΩ effective
/// - DRV8301 amp gain: 10 V/V (for Cheap FOCer 2 v1.0, 20 V/V for v0.9)
/// - ADC: 12-bit with 3.3V reference
/// - VBUS divider: 39k / 2.2k = (39 + 2.2) / 2.2 ratio
/// - Max continuous current: ~60A (FET rating, IRFS7530)
/// - Max VBUS: 57V (VESC HW_LIM_VIN, FET Vds=60V with margin)
#[cfg(feature = "board-cf2")]
pub const BOARD: BoardConfig = BoardConfig {
    calib: BoardCalib {
        shunt_ohms: 0.0005,                     // 0.5mΩ (two 1mΩ in parallel)
        amp_gain: 10.0,                         // DRV8301 10 V/V gain
        adc_vref_mv: 3300,                      // 3.3V
        adc_max_counts: 4095,                   // 12-bit
        invert_current_sign: false,             // DRV8301: standard polarity
        vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    },
    max_iq_target_a: 10.0, // Max torque current
    // Fault thresholds (matched to VESC hwconf)
    max_phase_current_a: 60.0, // Peak phase current limit (FET rating)
    max_vbus_mv: 57_000,       // Overvoltage at 57V (VESC HW_LIM_VIN)
    min_vbus_mv: 6_000,        // Undervoltage at 6V (VESC HW_LIM_VIN)
    max_fet_temp_c: 100.0,     // FET overtemp at 100°C
    max_motor_temp_c: 120.0,   // motor winding NTC on PC4
    // Phase-voltage sensing on PA0/PA1/PA2 (SENS1/2/3), divider = Vbus divider
    // (39k/2.2k). No RC phase filters on this board → usable only undriven
    // (back-EMF / coasting-rotation detection), not while PWMing.
    phase_sense: Some(PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: false,
    }),
};

/// VESC 6 MK5 (Trampa layout + clones; the bench unit is a Flipsky
/// "Mini V6 MK5") board configuration — facts from VESC hwconf
/// hw_60_core.h + the Flipsky listing, see docs/hw/vesc6-mk5.md.
///
/// Hardware specs:
/// - In-line phase shunts (HW_HAS_PHASE_SHUNTS): 0.5mΩ, standard polarity
/// - DRV8301 amp gain: 20 V/V (VESC CURRENT_AMP_GAIN, all hw60 marks)
/// - VBUS divider: 39k / 2.2k, same on SENS1-3
/// - Flipsky rating: 70A continuous / 200A instantaneous, 14-60V
///   (original Trampa hw60: ±120A, VIN 6-57V) — NB 14V operating minimum,
///   power the bench from ≥14V
#[cfg(feature = "board-vesc6-mk5")]
pub const BOARD: BoardConfig = BoardConfig {
    calib: BoardCalib {
        shunt_ohms: 0.0005, // 0.5mΩ in-line phase shunts
        amp_gain: 20.0,     // DRV8301 20 V/V gain (must match drv8301.rs SHUNT_AMP_GAIN)
        adc_vref_mv: 3300,
        adc_max_counts: 4095,
        invert_current_sign: false, // hw60 does NOT define INVERTED_SHUNT_POLARITY
        vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    },
    max_iq_target_a: 10.0,     // Max torque current
    max_phase_current_a: 70.0, // Flipsky continuous rating (200A "instantaneous" ignored)
    max_vbus_mv: 57_000,       // Overvoltage at 57V (VESC HW_LIM_VIN; vendor abs max 60V)
    min_vbus_mv: 6_000,        // UV fault floor (VESC HW_LIM_VIN; operating min is 14V)
    max_fet_temp_c: 100.0,     // Conservative (original hw60 cutoff: 110°C)
    max_motor_temp_c: 120.0,   // motor winding NTC on PC4
    // Phase-voltage sensing on PA0/PA1/PA2 (SENS1/2/3), divider = Vbus
    // divider. MK5 has switchable RC phase filters (enabled via PC13 at
    // boot) → usable while PWMing, unlike CF2.
    phase_sense: Some(PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: true,
    }),
};

/// JBLDC high-voltage IGBT inverter board configuration (`hw_jbldc.h` / `hw_jbldc.c`).
///
/// Hardware specs:
/// - Gate drivers: 6x Infineon EiceDRIVER (1ED38x0Mc12M) on I2C1 (PB6/PB7)
/// - Power stage: Infineon FS200 6-pack IGBT module, 3000 ns dead time
/// - Current sensors: MLX91208-CAH-000 IMC Hall sensors (`CURRENT_SHUNT_RES = 1.0`,
///   `CURRENT_AMP_GAIN = -0.00291842` V/A) on PC2 (IA), PC1 (IB), PC0 (IC)
/// - VBUS sense: PA0 (`ADC123_IN0`), 47:1 transfer function, 10–150V (`HW_LIM_VIN`)
/// - Phase voltage sense: PC4/PA2/PA1 (`SENS1/2/3`), 47:1 ratio, no RC phase filters
/// - Thermal sense: IGBT NTC on PB1 (`ADC12_IN9`, cubic transfer curve, 110°C trip) +
///   gate-driver PCB I2C sensor (`0x48` on I2C1); motor NTC is unwired (`NC`)
#[cfg(feature = "board-jbldc")]
pub const BOARD: BoardConfig = BoardConfig {
    calib: BoardCalib {
        shunt_ohms: 1.0,           // Unity R_shunt so amp_gain holds full V/A sensitivity
        amp_gain: 0.002_918_42,    // MLX91208-CAH-000 sensitivity (V/A)
        adc_vref_mv: 3300,         // 3.3V reference
        adc_max_counts: 4095,      // 12-bit ADC
        invert_current_sign: true, // Negative CURRENT_AMP_GAIN in hw_jbldc.h
        vbus_divider_ratio: 47.0,  // HVDC_TRANSFER_FUNCTION = 47
    },
    max_iq_target_a: 10.0,
    max_phase_current_a: 200.0, // FS200 6-pack IGBT module rating
    max_vbus_mv: 150_000,       // Overvoltage at 150V (HW_LIM_VIN)
    min_vbus_mv: 10_000,        // Undervoltage at 10V (HW_LIM_VIN)
    max_fet_temp_c: 110.0,      // IGBT overtemp at 110°C (HW_LIM_TEMP_FET)
    max_motor_temp_c: 0.0,      // Motor NTC unwired (ADC_IND_TEMP_MOTOR is NC)
    phase_sense: Some(PhaseSense {
        divider_ratio: 47.0, // PHASE_VOLTAGE_TRANSFER_FUNCTION = 47
        has_filters: false,
    }),
};

/// On-board PCB/FET NTC thermistor configuration (PA3, ADC123_IN3)
/// High-side 10k NTC with 10k pull-down resistor
/// From VESC: NTC_RES(adc_val) = (4095.0 * 10000.0) / adc_val - 10000.0
#[cfg(not(feature = "board-jbldc"))]
pub const NTC_BOARD: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,
    r0_ohm: 10_000.0,
    beta: 3380.0,
    t0_k: 273.15 + 25.0,
    topology: NtcTopology::HighSide,
};

/// Motor temperature NTC configuration (PC4, ADC12_IN14)
/// Low-side 10k NTC with 10k pull-up resistor (external motor thermistor)
/// From VESC: NTC_RES_MOTOR(adc_val) = 10000.0 / ((4095.0 / adc_val) - 1.0)
#[cfg(not(feature = "board-jbldc"))]
pub const NTC_MOTOR: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,
    r0_ohm: 10_000.0,
    beta: 3380.0, // Default, typically configured per motor
    t0_k: 273.15 + 25.0,
    topology: NtcTopology::LowSide,
};

/// Convert JBLDC IGBT module NTC raw ADC reading (PB1, `ADC12_IN9`) to 0.1°C units.
///
/// Uses the cubic transfer polynomial from `hw_jbldc.c` (`hw_get_IGBT_temp`),
/// failing closed (`NtcConfig::INVALID_TEMP_C_X10`) if the input hits either ADC rail.
#[cfg(feature = "board-jbldc")]
#[inline]
pub fn jbldc_igbt_temp_c_x10_from_adc(raw: u16, adc_max_counts: u16) -> i16 {
    if raw == 0 || raw >= adc_max_counts {
        return NtcConfig::INVALID_TEMP_C_X10;
    }
    let v = f32::from(raw) * (3.3 / f32::from(adc_max_counts));
    let v2 = v * v;
    let v3 = v2 * v;
    let temp_c = -3.82494 * v3 + 31.12876 * v2 - 98.58418 * v + 135.04544;
    if temp_c.is_finite() {
        (temp_c * 10.0) as i16
    } else {
        NtcConfig::INVALID_TEMP_C_X10
    }
}

// ============================================================================
// PWM Configuration
// ============================================================================

/// Motor PWM configuration
///
/// Central source of truth for PWM frequency and timing.
/// Used by motor.rs for timer setup and control/foc.rs for dt calculation.
#[cfg(not(feature = "board-jbldc"))]
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new().with_dead_time_ns(360);
// VESC HW_DEAD_TIME_NSEC fallback: 360ns — neither CF2 nor hw60 overrides it

/// Motor PWM configuration for JBLDC (`HW_DEAD_TIME_NSEC = 3000.0` for Infineon IGBT 6-pack).
#[cfg(feature = "board-jbldc")]
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new().with_dead_time_ns(3000);

// ============================================================================
// Timing Configuration
// ============================================================================

/// Core clock after PLL init — MUST match the RCC setup in
/// hardware/peripherals.rs (8 MHz HSE /4 ×168 /2 = 168 MHz).
/// Single source for all cycle-based timing: the ISR cycle budget,
/// busy-wait delays (DRV8301 tWAKE, bit-bang SPI), load percentages.
pub const CPU_HZ: u32 = 168_000_000;

// ============================================================================
// Protocol Configuration (CAN + NMEA + streaming-protocol)
// ============================================================================

pub use oxifoc_core::icd::{
    CAN_BITRATE, NMEA_INTER_FRAME_DELAY_MS, NMEA_RX_ARBITRATION_ID, NMEA_TX_ARBITRATION_ID,
};
