//! AS5147 / AS504x 14-bit magnetic rotary position sensor driver over SPI.
//!
//! Reference implementations:
//! - `https://github.com/jakestrj/rotary-sensor-board-rs` (`src/as5147/mod.rs`)
//! - VESC firmware `encoder/enc_as504x.c`
//!
//! # Hardware Wiring (JBLDC)
//! - `SPI1`: `SCK = PA5`, `MISO = PA6`, `MOSI = PA7` (AF5, SPI Mode 1: CPOL=0, CPHA=1)
//! - `CS` (`NSS`): `PC8` (active-low GPIO output)
//!
//! # Architecture & Real-Time Constraints
//! The 20 kHz FOC `ADC` ISR has a total budget of 8400 CPU cycles (50 µs at
//! 168 MHz). A 16-bit SPI transfer at 1 MHz takes ~16 µs per frame (~35 µs for
//! an angle read and ~120 µs when refreshing diagnostics), so SPI transactions
//! **never** run inside the `ADC` ISR or under a critical section.
//!
//! Instead, [`as5147_task`] runs in thread mode with interrupts enabled (so the
//! priority-0 `ADC` ISR preempts freely), executes the VESC `enc_as504x_routine`
//! state machine, and publishes angle/velocity + diagnostics into a shared
//! state read in O(1) by [`As5147AngleProxy`] during the FOC loop.

#![allow(dead_code)] // Public driver API and diagnostics fields exposed for protocol/telemetry

use core::cell::RefCell;
use core::f32::consts::TAU;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

#[cfg(feature = "board-jbldc")]
use embassy_stm32::gpio::{Level, Output, Speed};
#[cfg(feature = "board-jbldc")]
use embassy_stm32::mode::Blocking;
#[cfg(feature = "board-jbldc")]
use embassy_stm32::spi::{self, Spi};
#[cfg(feature = "board-jbldc")]
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
#[cfg(feature = "board-jbldc")]
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

use oxifoc_core::foc::hall_sensor::Direction;
use oxifoc_core::foc::sensors::{
    AngleSample, AngleSensor, EncoderSensorTrait, EncoderType, VelocitySensor,
};
use oxifoc_core::foc::{clamp_f32, wrap_angle};

#[cfg(feature = "board-jbldc")]
use crate::sensors::hall;

// ============================================================================
// Constants & Register Definitions
// ============================================================================

/// Counts per mechanical revolution (14-bit resolution = 16384).
pub const AS5147_CPR: u32 = 16_384;

/// Half-revolution counts for signed wrap-around detection.
const AS5147_HALF_CPR: i32 = (AS5147_CPR / 2) as i32;

/// 14-bit data payload mask (bits 13..0).
pub const DATA_MASK: u16 = 0x3FFF;

/// Error flag bit in MISO data frame (bit 14).
pub const ERROR_FLAG_BIT: u16 = 0x4000;

/// SPI read bit in MOSI command frame (bit 14 = 1).
const READ: u16 = 0x4000;

/// SPI write bit in MOSI command frame (bit 14 = 0).
const WRITE: u16 = 0x0000;

/// Precomputed SPI command words with even parity (matching `enc_as504x.c`):
/// - `0x3FFD | READ | PARITY` -> `0x7FFD` (`DIAAGC`)
/// - `0x3FFE | READ | PARITY` -> `0x7FFE` (`MAG`)
/// - `0x3FFF | READ | PARITY` -> `0xFFFF` (`ANGLECOM` / `NOP` readback)
/// - `0x0001 | READ | PARITY` -> `0x4001` (`ERRFL` clear error flag)
pub const READ_DIAAGC_MSG: u16 = 0x7FFD;
pub const READ_MAG_MSG: u16 = 0x7FFE;
pub const READ_NOP_MSG: u16 = 0xFFFF;
pub const READ_ANGLE_MSG: u16 = 0xFFFF;
pub const READ_CLEAR_ERROR_FLAG_MSG: u16 = 0x4001;

/// Refresh `DIAAGC` and `MAG` diagnostics every N angle samples (`enc_as504x.c`).
pub const REFRESH_DIAG_AFTER_NSAMPLES: u16 = 100;

/// Consecutive diagnostic failures before marking sensor disconnected (`enc_as504x.c`).
pub const CONNECTION_DETERMINATOR_ERROR_THRESHOLD: u32 = 5;

/// Timestamp staleness ceiling in µs before `As5147AngleProxy::sample` rejects
/// cached angle data (`AS504x_DATA_INVALID_THRESHOLD` = 20 000 µs in `enc_as504x.c`).
pub const DATA_INVALID_THRESHOLD_US: u64 = 20_000;

/// Default SPI1 clock frequency for AS5147 (1 MHz, Mode 1, matching `rotary-sensor-board-rs`).
pub const AS5147_SPI_FREQ_HZ: u32 = 1_000_000;

/// Default polling period in microseconds for [`as5147_task`] (500 µs = 2 kHz).
pub const AS5147_POLL_PERIOD_US: u64 = 500;

/// Inter-frame CS high delay in CPU cycles at 168 MHz.
/// AS5147 datasheet specifies $t_{CSn} \ge 350\text{ ns}$ between frames;
/// 64 cycles at 168 MHz is ~380 ns.
const CS_INTERFRAME_DELAY_CYCLES: u32 = 64;

/// Short CS setup/hold delay in CPU cycles (~60 ns at 168 MHz).
const CS_SETUP_HOLD_DELAY_CYCLES: u32 = 10;

/// Minimum angular speed (rad/s) to consider the rotor moving rather than stopped.
const MIN_MOVING_OMEGA_RAD_S: f32 = 0.5;

/// Low-pass filter time constant for velocity estimation (seconds).
const VELOCITY_LPF_TAU_S: f32 = 0.002;

/// Maximum forward extrapolation horizon in seconds when the 20 kHz FOC ISR
/// interpolates between background SPI samples.
const MAX_EXTRAPOLATION_DT_S: f32 = 0.002;

/// AS5147 register addresses (`rotary-sensor-board-rs/src/as5147/mod.rs`).
#[repr(u16)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum Register {
    /// Error register (`ERRFL`, `0x0001`). Reading clears error flags.
    ClearErrorFlag = 0x0001,
    /// Programming control register (`PROG`, `0x0003`).
    ProgrammingControl = 0x0003,
    /// OTP zero position high byte (`ZPOSM`, `0x0016`).
    OtpRegisterZeroPosHigh = 0x0016,
    /// OTP zero position low 6 bits + comp flags (`ZPOSL`, `0x0017`).
    OtpRegisterZeroPosLow = 0x0017,
    /// Diagnostics and Automatic Gain Control (`DIAAGC`, `0x3FFD`).
    DiagAgc = 0x3FFD,
    /// CORDIC magnitude (`MAG`, `0x3FFE`).
    Magnitude = 0x3FFE,
    /// Compensated angle (`ANGLECOM`, `0x3FFF`).
    Angle = 0x3FFF,
}

/// Errors returned by direct [`As5147`] register transactions.
#[allow(clippy::enum_variant_names)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum Error<SPIError, CSError> {
    /// Underlying SPI bus error.
    Spi(SPIError),
    /// Chip-select GPIO error.
    Cs(CSError),
    /// Received frame failed even parity check.
    Parity,
    /// Sensor set the `EF` (Error Flag, bit 14) on the response frame.
    ErrorFlag,
}

// ============================================================================
// Parity Helpers
// ============================================================================

/// Compute and set bit 15 so the 16-bit word has even parity (`rotary-sensor-board-rs`).
#[inline]
pub const fn set_parity(par: u16) -> u16 {
    let payload = par & 0x7FFF;
    if payload.count_ones() % 2 == 1 {
        payload | 0x8000
    } else {
        payload
    }
}

/// Return `true` if the 16-bit word has even parity across all 16 bits
/// (`AS504x_check_parity` in `enc_as504x.c` / `check_parity` in `rotary-sensor-board-rs`).
#[inline]
pub const fn check_parity(x: u16) -> bool {
    x.count_ones().is_multiple_of(2)
}

// ============================================================================
// Diagnostic & State Structures (`enc_as504x.c` + `rotary-sensor-board-rs`)
// ============================================================================

/// Decoded `DIAAGC` (`0x3FFD`) register (`rotary-sensor-board-rs` & `enc_as504x.c`).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, defmt::Format)]
pub struct DiagAgc {
    /// 8-bit Automatic Gain Control value (0 = strong field, 255 = weak field).
    pub agc: u8,
    /// Offset Compensation Finished (`OCF`, bit 8).
    pub is_ocf: bool,
    /// CORDIC Overflow (`COF`, bit 9).
    pub is_cof: bool,
    /// Magnetic field too strong (`COMP_L`, bit 10).
    pub is_comp_low: bool,
    /// Magnetic field too weak / missing magnet (`COMP_H`, bit 11).
    pub is_comp_high: bool,
}

impl DiagAgc {
    /// Decode a raw 16-bit `DIAAGC` response frame.
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self {
            agc: (raw & 0x00FF) as u8,
            is_ocf: (raw & 0x0100) != 0,
            is_cof: (raw & 0x0200) != 0,
            is_comp_low: (raw & 0x0400) != 0,
            is_comp_high: (raw & 0x0800) != 0,
        }
    }
}

/// Complete sensor diagnostic telemetry matching VESC `AS504x_diag` (`encoder_datatype.h`).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, defmt::Format)]
pub struct As5147Diagnostics {
    /// True when SPI communication and diagnostic validation succeed.
    pub is_connected: bool,
    /// Automatic Gain Control value (`DIAAGC[7:0]`).
    pub agc_value: u8,
    /// 14-bit CORDIC magnitude (`MAG[13:0]`).
    pub magnitude: u16,
    /// Offset Compensation Finished flag (`DIAAGC[8]`).
    pub is_ocf: bool,
    /// CORDIC Overflow flag (`DIAAGC[9]`).
    pub is_cof: bool,
    /// Magnetic field too strong flag (`DIAAGC[10]`).
    pub is_comp_low: bool,
    /// Magnetic field too weak / no magnet flag (`DIAAGC[11]`).
    pub is_comp_high: bool,
    /// Last raw `DIAAGC` word received over SPI.
    pub serial_diag_flgs: u16,
    /// Last raw `MAG` word received over SPI.
    pub serial_magnitude: u16,
    /// Last raw `ERRFL` word received over SPI.
    pub serial_error_flags: u16,
}

impl As5147Diagnostics {
    /// Verify that raw diagnostic words are plausible (`AS504x_verify_serial` in `enc_as504x.c`).
    ///
    /// Rejects:
    /// - Both `COMP_H` and `COMP_L` asserted simultaneously (impossible in hardware, e.g. MISO pulled high).
    /// - Both `magnitude == 0` and `agc_value == 0` (catches all-zero MISO disconnect, which otherwise
    ///   passes even-parity check).
    #[inline]
    pub const fn verify_serial(&self) -> bool {
        let diag = DiagAgc::from_raw(self.serial_diag_flgs);
        let test_magnitude = self.serial_magnitude & DATA_MASK;

        if diag.is_comp_high && diag.is_comp_low {
            return false;
        }
        if (test_magnitude.wrapping_add(diag.agc as u16)) == 0 {
            return false;
        }
        true
    }

    /// Populate decoded diagnostic fields from `serial_diag_flgs` and `serial_magnitude`
    /// (`AS504x_deserialize_diag` in `enc_as504x.c`).
    #[inline]
    pub fn deserialize_diag(&mut self) {
        let diag = DiagAgc::from_raw(self.serial_diag_flgs);
        self.agc_value = diag.agc;
        self.is_ocf = diag.is_ocf;
        self.is_cof = diag.is_cof;
        self.is_comp_low = diag.is_comp_low;
        self.is_comp_high = diag.is_comp_high;
        self.magnitude = self.serial_magnitude & DATA_MASK;
    }
}

/// Operational state of the AS5147 sensor matching VESC `AS504x_state` (`encoder_datatype.h`).
#[derive(Copy, Clone, Debug, Default, defmt::Format)]
pub struct As5147State {
    /// Sample counter for periodic diagnostic refresh (`0..REFRESH_DIAG_AFTER_NSAMPLES`).
    pub diag_fetch_now_count: u16,
    /// Saturating counter of consecutive diagnostic failures (`0..=CONNECTION_DETERMINATOR_ERROR_THRESHOLD`).
    pub spi_communication_error_count: u32,
    /// True if the last angle read had the SPI `EF` bit set (bit 14 = 1).
    pub spi_data_err_raised: bool,
    /// Latest raw 14-bit angle value (`0..16384`).
    pub spi_val: u16,
    /// Latest mechanical rotor angle in degrees (`[0.0, 360.0)`).
    pub last_enc_angle_deg: f32,
    /// Latest mechanical rotor angle in radians (`[0.0, 2π)`).
    pub last_enc_angle_rad: f32,
    /// Cumulative SPI parity/frame error count.
    pub spi_error_cnt: u32,
    /// Low-pass filtered SPI error rate (`[0.0, 1.0]`).
    pub spi_error_rate: f32,
    /// Cumulative missing-magnet (`COMP_H`) error count.
    pub encoder_no_magnet_error_cnt: u32,
    /// Low-pass filtered missing-magnet error rate (`[0.0, 1.0]`).
    pub encoder_no_magnet_error_rate: f32,
    /// Decoded sensor diagnostics (`DIAAGC`, `MAG`, `ERRFL`, connectivity).
    pub sensor_diag: As5147Diagnostics,
}

impl As5147State {
    /// Update connection state based on whether the latest diagnostic check succeeded
    /// (`AS504x_determinate_if_connected` in `enc_as504x.c`).
    #[inline]
    pub fn determinate_if_connected(&mut self, was_last_valid: bool) {
        if !was_last_valid {
            self.spi_communication_error_count =
                self.spi_communication_error_count.saturating_add(1);
            if self.spi_communication_error_count >= CONNECTION_DETERMINATOR_ERROR_THRESHOLD {
                self.spi_communication_error_count = CONNECTION_DETERMINATOR_ERROR_THRESHOLD;
                self.sensor_diag.is_connected = false;
            }
        } else if self.spi_communication_error_count > 0 {
            self.spi_communication_error_count -= 1;
        } else {
            self.sensor_diag.is_connected = true;
        }
    }
}

// ============================================================================
// Low-Level SPI Driver (`As5147<SPI, CS>`)
// ============================================================================

/// First-order low-pass filter step matching VESC `UTILS_LP_FAST(value, sample, filter_constant)`.
#[inline]
fn utils_lp_fast(value: &mut f32, sample: f32, filter_constant: f32) {
    let alpha = clamp_f32(filter_constant, 0.0, 1.0);
    *value -= alpha * (*value - sample);
}

/// AS5147 magnetic rotary encoder driver over `embedded_hal::spi::SpiBus<u8>`.
pub struct As5147<SPI, CS> {
    spi: SPI,
    cs: CS,
    state: As5147State,
}

impl<SPI, CS, SPIError, CSError> As5147<SPI, CS>
where
    SPI: SpiBus<u8, Error = SPIError>,
    CS: OutputPin<Error = CSError>,
{
    /// Create a new AS5147 driver owning the SPI bus and chip-select pin.
    pub fn new(spi: SPI, mut cs: CS) -> Self {
        let _ = cs.set_high();
        Self {
            spi,
            cs,
            state: As5147State::default(),
        }
    }

    /// Borrow the current [`As5147State`].
    #[inline]
    pub const fn state(&self) -> &As5147State {
        &self.state
    }

    /// Borrow the current [`As5147Diagnostics`].
    #[inline]
    pub const fn diagnostics(&self) -> &As5147Diagnostics {
        &self.state.sensor_diag
    }

    /// Perform a single 16-bit full-duplex SPI frame with CS assertion and setup/hold delays.
    fn transfer_frame(&mut self, tx_word: u16) -> Result<u16, Error<SPIError, CSError>> {
        let tx = tx_word.to_be_bytes();
        let mut rx = [0u8; 2];

        self.cs.set_low().map_err(Error::Cs)?;
        cortex_m::asm::delay(CS_SETUP_HOLD_DELAY_CYCLES);
        let res = self.spi.transfer(&mut rx, &tx);
        cortex_m::asm::delay(CS_SETUP_HOLD_DELAY_CYCLES);
        let cs_res = self.cs.set_high();

        res.map_err(Error::Spi)?;
        cs_res.map_err(Error::Cs)?;

        Ok(u16::from_be_bytes(rx))
    }

    /// Send a 16-bit command frame (write-only) with CS assertion.
    fn send_frame(&mut self, tx_word: u16) -> Result<(), Error<SPIError, CSError>> {
        let tx = tx_word.to_be_bytes();

        self.cs.set_low().map_err(Error::Cs)?;
        cortex_m::asm::delay(CS_SETUP_HOLD_DELAY_CYCLES);
        let res = self.spi.write(&tx);
        let _ = self.spi.flush();
        cortex_m::asm::delay(CS_SETUP_HOLD_DELAY_CYCLES);
        let cs_res = self.cs.set_high();

        res.map_err(Error::Spi)?;
        cs_res.map_err(Error::Cs)?;

        Ok(())
    }

    /// Read any 14-bit register using the standard two-frame command + readback sequence.
    ///
    /// Checks both even parity (bit 15) and the command Error Flag (`EF`, bit 14).
    pub fn read_reg(&mut self, reg: Register) -> Result<u16, Error<SPIError, CSError>> {
        let cmd = set_parity(READ | (reg as u16));
        self.send_frame(cmd)?;
        cortex_m::asm::delay(CS_INTERFRAME_DELAY_CYCLES);
        let response = self.transfer_frame(READ_NOP_MSG)?;

        if !check_parity(response) {
            return Err(Error::Parity);
        }
        if (response & ERROR_FLAG_BIT) != 0 {
            return Err(Error::ErrorFlag);
        }
        Ok(response & DATA_MASK)
    }

    /// Read the compensated 14-bit angle (`ANGLECOM`, `0x3FFF`).
    #[inline]
    pub fn angle(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_reg(Register::Angle)
    }

    /// Read the 14-bit CORDIC magnitude (`MAG`, `0x3FFE`).
    #[inline]
    pub fn magnitude(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_reg(Register::Magnitude)
    }

    /// Read and decode the `DIAAGC` (`0x3FFD`) diagnostics and AGC register.
    #[inline]
    pub fn diag_gain(&mut self) -> Result<DiagAgc, Error<SPIError, CSError>> {
        let raw = self.read_reg(Register::DiagAgc)?;
        Ok(DiagAgc::from_raw(raw))
    }

    /// Read the `ERRFL` (`0x0001`) register to clear sticky sensor error flags
    /// (`AS504x_fetch_clear_err_diag` in `enc_as504x.c`).
    pub fn clear_error_flag(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.send_frame(READ_CLEAR_ERROR_FLAG_MSG)?;
        cortex_m::asm::delay(CS_INTERFRAME_DELAY_CYCLES);
        let raw = self.transfer_frame(READ_NOP_MSG)?;
        self.state.sensor_diag.serial_error_flags = raw;
        if !check_parity(raw) {
            return Err(Error::Parity);
        }
        Ok(raw & DATA_MASK)
    }

    /// Pipelined fetch of `DIAAGC` (`0x3FFD`) and `MAG` (`0x3FFE`) matching
    /// `AS504x_fetch_diag` in `enc_as504x.c`.
    ///
    /// Returns `true` if both received words have `EF == 0` and valid even parity.
    fn fetch_diag(&mut self) -> Result<bool, Error<SPIError, CSError>> {
        // Frame 1: request DIAAGC (0x3FFD)
        self.send_frame(READ_DIAAGC_MSG)?;
        cortex_m::asm::delay(CS_INTERFRAME_DELAY_CYCLES);

        // Frame 2: clock out DIAAGC while requesting MAG (0x3FFE)
        let diag_flgs = self.transfer_frame(READ_MAG_MSG)?;
        cortex_m::asm::delay(CS_INTERFRAME_DELAY_CYCLES);

        // Frame 3: clock out MAG while sending NOP / ANGLECOM (0x3FFF)
        let magnitude = self.transfer_frame(READ_NOP_MSG)?;

        self.state.sensor_diag.serial_diag_flgs = diag_flgs;
        self.state.sensor_diag.serial_magnitude = magnitude;

        let no_ef = ((diag_flgs | magnitude) & ERROR_FLAG_BIT) == 0;
        let parity_ok = check_parity(diag_flgs) && check_parity(magnitude);
        Ok(no_ef && parity_ok)
    }

    /// Execute one iteration of the VESC `enc_as504x_routine` state machine (`enc_as504x.c`).
    ///
    /// - Reads the 14-bit compensated angle (`0x3FFF`).
    /// - Periodically (every [`REFRESH_DIAG_AFTER_NSAMPLES`] samples) or immediately when
    ///   `spi_data_err_raised` is set, clears error flags and fetches `DIAAGC` + `MAG`
    ///   diagnostics to validate sensor connectivity and magnet presence.
    /// - Updates low-pass filtered error rates (`spi_error_rate`, `encoder_no_magnet_error_rate`).
    /// - Returns `Ok(Some(raw_angle_14bit))` if this sample's angle frame was valid and the
    ///   sensor is connected with a magnet present, or `Ok(None)` if invalid/disconnected.
    pub fn routine(&mut self, dt_s: f32) -> Result<Option<u16>, Error<SPIError, CSError>> {
        let timestep = if dt_s > 0.0 { dt_s } else { 0.0005 };

        // 1. Read ANGLECOM (0x3FFF): command frame + inter-frame delay + readback frame
        self.send_frame(READ_ANGLE_MSG)?;
        cortex_m::asm::delay(CS_INTERFRAME_DELAY_CYCLES);
        let pos = self.transfer_frame(READ_NOP_MSG)?;

        self.state.diag_fetch_now_count = self.state.diag_fetch_now_count.wrapping_add(1);

        // 2. Refresh diagnostics every 100 samples or when a command error flag was raised
        if self.state.diag_fetch_now_count >= REFRESH_DIAG_AFTER_NSAMPLES
            || self.state.spi_data_err_raised
        {
            // Clear previous error flags before fetching fresh diagnostics
            let _ = self.clear_error_flag();
            cortex_m::asm::delay(CS_INTERFRAME_DELAY_CYCLES);

            let diag_ok = self.fetch_diag()?;
            if diag_ok {
                if self.state.sensor_diag.verify_serial() {
                    self.state.sensor_diag.deserialize_diag();
                    self.state.determinate_if_connected(true);
                } else {
                    self.state.determinate_if_connected(false);
                }
            } else {
                self.state.determinate_if_connected(false);
            }

            self.state.diag_fetch_now_count = 0;
        }

        // 3. Validate angle frame parity, EF bit, connection status, and magnet presence
        self.state.spi_data_err_raised = (pos & ERROR_FLAG_BIT) != 0;

        if check_parity(pos)
            && !self.state.spi_data_err_raised
            && self.state.sensor_diag.is_connected
        {
            utils_lp_fast(&mut self.state.spi_error_rate, 0.0, timestep);

            if self.state.sensor_diag.is_comp_high {
                self.state.encoder_no_magnet_error_cnt =
                    self.state.encoder_no_magnet_error_cnt.wrapping_add(1);
                utils_lp_fast(&mut self.state.encoder_no_magnet_error_rate, 1.0, timestep);
                Ok(None)
            } else {
                let raw_14 = pos & DATA_MASK;
                self.state.spi_val = raw_14;
                let ratio = f32::from(raw_14) / (AS5147_CPR as f32);
                self.state.last_enc_angle_deg = ratio * 360.0;
                self.state.last_enc_angle_rad = ratio * TAU;
                utils_lp_fast(&mut self.state.encoder_no_magnet_error_rate, 0.0, timestep);
                Ok(Some(raw_14))
            }
        } else {
            self.state.spi_error_cnt = self.state.spi_error_cnt.wrapping_add(1);
            utils_lp_fast(&mut self.state.spi_error_rate, 1.0, timestep);
            Ok(None)
        }
    }
}

// ============================================================================
// Electrical Angle & Velocity Estimator + Shared State for FOC Loop
// ============================================================================

/// Electrical angle and velocity estimator driven by 14-bit AS5147 samples.
struct As5147Estimator {
    /// Latest valid electrical angle sample (`angle` in `[0, 2π)`, `omega` in `rad/s`).
    latest_sample: Option<AngleSample>,
    /// Timestamp (in `hall::now_ticks()` 1 MHz µs domain) of the latest valid angle sample.
    last_sample_ticks: u64,
    /// Previous raw 14-bit count (`0..16384`) for unwrapping mechanical velocity.
    prev_raw: Option<u16>,
    /// Filtered mechanical angular velocity (rad/s).
    mech_omega_rad_s: f32,
    /// Electrical zero offset in radians (`[0, 2π)`).
    offset_rad: f32,
    /// Direction inversion flag (`true` = invert mechanical angle).
    inverted: bool,
    /// Latest sensor state and diagnostics snapshot.
    state: As5147State,
}

impl As5147Estimator {
    const fn new() -> Self {
        Self {
            latest_sample: None,
            last_sample_ticks: 0,
            prev_raw: None,
            mech_omega_rad_s: 0.0,
            offset_rad: 0.0,
            inverted: false,
            state: As5147State {
                diag_fetch_now_count: REFRESH_DIAG_AFTER_NSAMPLES, // Force immediate diag fetch on first sample
                spi_communication_error_count: 0,
                spi_data_err_raised: false,
                spi_val: 0,
                last_enc_angle_deg: 0.0,
                last_enc_angle_rad: 0.0,
                spi_error_cnt: 0,
                spi_error_rate: 0.0,
                encoder_no_magnet_error_cnt: 0,
                encoder_no_magnet_error_rate: 0.0,
                sensor_diag: As5147Diagnostics {
                    is_connected: false,
                    agc_value: 0,
                    magnitude: 0,
                    is_ocf: false,
                    is_cof: false,
                    is_comp_low: false,
                    is_comp_high: false,
                    serial_diag_flgs: 0,
                    serial_magnitude: 0,
                    serial_error_flags: 0,
                },
            },
        }
    }

    /// Update the estimator with a fresh 14-bit raw angle at `now_ticks` (1 MHz µs).
    fn update_sample(&mut self, raw_14: u16, now_ticks: u64, pole_pairs: u8, state: As5147State) {
        self.state = state;
        let pp = f32::from(pole_pairs.max(1));

        // Compute unwrapped mechanical velocity if we have a previous sample
        if let Some(prev) = self.prev_raw {
            let dt_ticks = now_ticks.wrapping_sub(self.last_sample_ticks);
            if dt_ticks > 0 && dt_ticks <= DATA_INVALID_THRESHOLD_US {
                let dt_s = (dt_ticks as f32) * 1.0e-6;
                let mut diff = i32::from(raw_14) - i32::from(prev);
                if diff > AS5147_HALF_CPR {
                    diff -= AS5147_CPR as i32;
                } else if diff < -AS5147_HALF_CPR {
                    diff += AS5147_CPR as i32;
                }
                if self.inverted {
                    diff = -diff;
                }
                let delta_mech_rad = (diff as f32) * (TAU / (AS5147_CPR as f32));
                let inst_mech_omega = delta_mech_rad / dt_s;
                let alpha = dt_s / (VELOCITY_LPF_TAU_S + dt_s);
                self.mech_omega_rad_s += alpha * (inst_mech_omega - self.mech_omega_rad_s);
            } else {
                self.mech_omega_rad_s = 0.0;
            }
        } else {
            self.mech_omega_rad_s = 0.0;
        }

        self.prev_raw = Some(raw_14);
        self.last_sample_ticks = now_ticks;

        // Compute mechanical angle in [0, 2π)
        let mut mech_rad = f32::from(raw_14) * (TAU / (AS5147_CPR as f32));
        if self.inverted {
            mech_rad = TAU - mech_rad;
        }

        // Convert to electrical angle in [0, 2π) and electrical angular velocity
        let elec_angle = wrap_angle(mech_rad * pp - self.offset_rad);
        let elec_omega = self.mech_omega_rad_s * pp;
        let direction = if elec_omega > MIN_MOVING_OMEGA_RAD_S {
            Direction::CounterClockwise
        } else if elec_omega < -MIN_MOVING_OMEGA_RAD_S {
            Direction::Clockwise
        } else {
            Direction::Stopped
        };

        self.latest_sample = Some(AngleSample {
            angle: elec_angle,
            omega: elec_omega,
            direction,
        });
    }

    /// Update diagnostic state when a sample was invalid or sensor disconnected.
    fn update_invalid(&mut self, state: As5147State) {
        self.state = state;
        if !state.sensor_diag.is_connected || state.sensor_diag.is_comp_high {
            self.latest_sample = None;
            self.prev_raw = None;
            self.mech_omega_rad_s = 0.0;
        }
    }

    /// Read the current electrical [`AngleSample`] extrapolated to `now_ticks`.
    #[inline]
    fn sample_at(&self, now_ticks: u64) -> Option<AngleSample> {
        if !self.state.sensor_diag.is_connected || self.state.sensor_diag.is_comp_high {
            return None;
        }
        let mut sample = self.latest_sample?;
        let age_ticks = now_ticks.wrapping_sub(self.last_sample_ticks);
        if age_ticks > DATA_INVALID_THRESHOLD_US {
            return None;
        }
        let dt_s = clamp_f32((age_ticks as f32) * 1.0e-6, 0.0, MAX_EXTRAPOLATION_DT_S);
        sample.angle = wrap_angle(sample.angle + sample.omega * dt_s);
        Some(sample)
    }
}

static ESTIMATOR: CriticalSectionMutex<RefCell<As5147Estimator>> =
    CriticalSectionMutex::new(RefCell::new(As5147Estimator::new()));

/// Configured motor pole pairs (synced from `foc::init` and runtime config).
static POLE_PAIRS: AtomicU8 = AtomicU8::new(7);

/// Configured counts per electrical revolution (derived from `AS5147_CPR / pole_pairs`
/// or explicitly set via [`EncoderSensorTrait::set_counts_per_electrical_rev`]).
static COUNTS_PER_ELEC_REV: AtomicU32 = AtomicU32::new(AS5147_CPR / 7);

/// Number of valid angle samples acquired since last 1 Hz stats log.
pub static SAMPLES_OK: AtomicU32 = AtomicU32::new(0);

/// Number of failed/rejected angle samples since last 1 Hz stats log.
pub static SAMPLES_ERR: AtomicU32 = AtomicU32::new(0);

/// Cumulative error count exposed via [`AngleSensor::error_count`] (can be reset by
/// [`AngleSensor::reset_errors`]).
static ERROR_COUNT: AtomicU32 = AtomicU32::new(0);

/// Set the motor pole-pair count used to convert mechanical angle to electrical angle.
#[inline]
pub fn set_pole_pairs(pole_pairs: u8) {
    if pole_pairs > 0 {
        POLE_PAIRS.store(pole_pairs, Ordering::Relaxed);
        COUNTS_PER_ELEC_REV.store(
            (AS5147_CPR / u32::from(pole_pairs)).max(1),
            Ordering::Relaxed,
        );
    }
}

/// Configure electrical zero offset (in radians) and direction inversion.
pub fn set_calibration(offset_rad: f32, inverted: bool) {
    ESTIMATOR.lock(|cell| {
        let mut est = cell.borrow_mut();
        est.offset_rad = wrap_angle(offset_rad);
        est.inverted = inverted;
    });
}

/// Return a snapshot of the latest [`As5147State`] (including [`As5147Diagnostics`]).
pub fn get_state() -> As5147State {
    ESTIMATOR.lock(|cell| cell.borrow().state)
}

/// Return a snapshot of the latest [`As5147Diagnostics`].
pub fn get_diagnostics() -> As5147Diagnostics {
    ESTIMATOR.lock(|cell| cell.borrow().state.sensor_diag)
}

// ============================================================================
// `As5147AngleProxy` — `AngleSensor` & `EncoderSensorTrait` for `PhaseManager`
// ============================================================================

/// Lightweight handle implementing [`AngleSensor`], [`VelocitySensor`], and
/// [`EncoderSensorTrait`] for [`oxifoc_core::foc::phase::PhaseManager`].
///
/// Safe and fast to call inside the 20 kHz FOC `ADC` ISR: reads pre-computed
/// state updated by [`as5147_task`] without touching the SPI bus.
#[derive(Copy, Clone, Debug, Default)]
pub struct As5147AngleProxy;

impl As5147AngleProxy {
    /// Create a new proxy reading from the shared AS5147 estimator state.
    #[inline]
    pub const fn new() -> Self {
        Self
    }
}

impl AngleSensor for As5147AngleProxy {
    #[inline]
    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        ESTIMATOR.lock(|cell| cell.borrow().sample_at(now_ticks))
    }

    #[inline]
    fn error_count(&self) -> u32 {
        ERROR_COUNT.load(Ordering::Relaxed)
    }

    #[inline]
    fn reset_errors(&mut self) {
        ERROR_COUNT.store(0, Ordering::Relaxed);
    }

    #[inline]
    fn is_present(&self) -> bool {
        cfg!(feature = "board-jbldc")
    }
}

impl VelocitySensor for As5147AngleProxy {
    #[inline]
    fn read_velocity(&self) -> f32 {
        ESTIMATOR.lock(|cell| {
            let est = cell.borrow();
            est.latest_sample.map_or(0.0, |s| s.omega)
        })
    }
}

impl EncoderSensorTrait for As5147AngleProxy {
    #[inline]
    fn counts(&self) -> i32 {
        ESTIMATOR.lock(|cell| i32::from(cell.borrow().state.spi_val))
    }

    fn set_zero(&mut self) {
        let pp = f32::from(POLE_PAIRS.load(Ordering::Relaxed).max(1));
        ESTIMATOR.lock(|cell| {
            let mut est = cell.borrow_mut();
            let raw = est.state.spi_val;
            let mut mech_rad = f32::from(raw) * (TAU / (AS5147_CPR as f32));
            if est.inverted {
                mech_rad = TAU - mech_rad;
            }
            est.offset_rad = wrap_angle(mech_rad * pp);
        });
    }

    #[inline]
    fn set_offset(&mut self, offset_rad: f32) {
        ESTIMATOR.lock(|cell| {
            cell.borrow_mut().offset_rad = wrap_angle(offset_rad);
        });
    }

    #[inline]
    fn offset(&self) -> f32 {
        ESTIMATOR.lock(|cell| cell.borrow().offset_rad)
    }

    #[inline]
    fn counts_per_electrical_rev(&self) -> u32 {
        COUNTS_PER_ELEC_REV.load(Ordering::Relaxed)
    }

    #[inline]
    fn set_counts_per_electrical_rev(&mut self, cpr: u32) {
        if cpr > 0 {
            COUNTS_PER_ELEC_REV.store(cpr, Ordering::Relaxed);
            let pp = (AS5147_CPR / cpr).clamp(1, u32::from(u8::MAX)) as u8;
            POLE_PAIRS.store(pp, Ordering::Relaxed);
        }
    }

    #[inline]
    fn index_seen(&self) -> bool {
        // AS5147 is an absolute 14-bit magnetic encoder — position is valid as
        // soon as SPI communication is verified and a magnet is present.
        ESTIMATOR.lock(|cell| {
            let diag = &cell.borrow().state.sensor_diag;
            diag.is_connected && !diag.is_comp_high
        })
    }

    fn reset_index(&mut self) {
        ESTIMATOR.lock(|cell| {
            let mut est = cell.borrow_mut();
            est.latest_sample = None;
            est.prev_raw = None;
            est.mech_omega_rad_s = 0.0;
        });
    }

    #[inline]
    fn encoder_type(&self) -> EncoderType {
        EncoderType::Absolute
    }
}

// ============================================================================
// STM32F405 Hardware Initialization & Background Polling Task
// ============================================================================

/// Concrete AS5147 driver type on STM32F405 `SPI1` (`PA5`/`PA6`/`PA7`) + `PC8` CS.
#[cfg(feature = "board-jbldc")]
pub type F405As5147 = As5147<Spi<'static, Blocking, spi::mode::Master>, Output<'static>>;

/// Initialize `SPI1` (`PA5` SCK, `PA6` MISO, `PA7` MOSI) and `PC8` CS for the AS5147
/// position sensor (`hw_jbldc.h` & `rotary-sensor-board-rs/src/main.rs`).
#[cfg(feature = "board-jbldc")]
pub fn init_as5147(r: crate::hardware::EncoderResources) -> F405As5147 {
    let mut spi_cfg = spi::Config::default();
    spi_cfg.mode = spi::MODE_1; // CPOL = 0 (IdleLow), CPHA = 1 (CaptureOnSecondTransition)
    spi_cfg.frequency = Hertz(AS5147_SPI_FREQ_HZ);

    let spi = Spi::new_blocking(r.spi1, r.pa5, r.pa7, r.pa6, spi_cfg);
    let cs = Output::new(r.pc8, Level::High, Speed::VeryHigh);

    let mut driver = As5147::new(spi, cs);
    // Force an immediate diagnostic fetch on the first routine() call
    driver.state.diag_fetch_now_count = REFRESH_DIAG_AFTER_NSAMPLES;

    defmt::info!(
        "AS5147: SPI1 initialized @ {} Hz (Mode 1), SCK=PA5 MISO=PA6 MOSI=PA7 CS=PC8",
        AS5147_SPI_FREQ_HZ
    );

    driver
}

/// Background task polling the AS5147 encoder over SPI1 and updating the
/// shared estimator for the 20 kHz FOC loop.
///
/// Runs in thread mode with interrupts enabled so the priority-0 `ADC` ISR
/// can preempt SPI transfers with zero jitter.
#[cfg(feature = "board-jbldc")]
#[embassy_executor::task]
pub async fn as5147_task(mut sensor: F405As5147) {
    let mut ticker = Ticker::every(Duration::from_micros(AS5147_POLL_PERIOD_US));
    let dt_s = (AS5147_POLL_PERIOD_US as f32) * 1.0e-6;
    let mut prev_connected = false;

    loop {
        ticker.next().await;

        let now_ticks = hall::now_ticks();
        let pole_pairs = POLE_PAIRS.load(Ordering::Relaxed);

        match sensor.routine(dt_s) {
            Ok(Some(raw_14)) => {
                SAMPLES_OK.fetch_add(1, Ordering::Relaxed);
                let state = *sensor.state();
                ESTIMATOR.lock(|cell| {
                    cell.borrow_mut()
                        .update_sample(raw_14, now_ticks, pole_pairs, state);
                });
            }
            Ok(None) | Err(_) => {
                SAMPLES_ERR.fetch_add(1, Ordering::Relaxed);
                ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                let state = *sensor.state();
                ESTIMATOR.lock(|cell| {
                    cell.borrow_mut().update_invalid(state);
                });
            }
        }

        let is_connected = sensor.diagnostics().is_connected;
        if is_connected != prev_connected {
            prev_connected = is_connected;
            let diag = sensor.diagnostics();
            if is_connected {
                defmt::info!(
                    "AS5147 connected: raw={} deg={=f32} mag={} agc={} ocf={} cof={} comp_l={} comp_h={}",
                    sensor.state().spi_val,
                    sensor.state().last_enc_angle_deg,
                    diag.magnitude,
                    diag.agc_value,
                    diag.is_ocf,
                    diag.is_cof,
                    diag.is_comp_low,
                    diag.is_comp_high,
                );
            } else {
                defmt::warn!(
                    "AS5147 disconnected: err_cnt={} diag_raw=0x{:04x} mag_raw=0x{:04x}",
                    sensor.state().spi_error_cnt,
                    diag.serial_diag_flgs,
                    diag.serial_magnitude,
                );
            }
        }
    }
}
