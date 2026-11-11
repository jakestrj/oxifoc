//! Platform-specific fault types for STM32F405 boards (DRV8301 / EiceDRIVER)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};
use oxifoc_core::foc::hall_sensor::HallFaultKind;

// Import DRV8301 fault status from driver crate
#[cfg(not(feature = "board-jbldc"))]
pub use drv8301_dd::FaultStatus as DrvFaultStatus;

/// Compact fault snapshot across the 6 Infineon EiceDRIVER gate drivers
/// (switch order: UH, UL, VH, VL, WH, WL at I2C addresses `0x0E..=0x13`).
#[cfg(feature = "board-jbldc")]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, defmt::Format)]
pub struct EiceDriverFaultStatus {
    /// Bitmask of switches (bits 0..6 = UH..WL) whose I2C status read failed.
    pub i2c_err_mask: u8,
    /// Bitmask of switches (bits 0..6 = UH..WL) with `RDYSTAT[2:0] != 0b111`.
    pub unready_mask: u8,
    /// Index (`0..6`) of the first switch reporting a fault or unready state.
    pub first_switch: u8,
    /// Raw `RDYSTAT[2:0]` of `first_switch` (bit0=CHIP_RDY, bit1=PRI_RDY, bit2=SEC_RDY).
    pub rdy_stat: u8,
    /// Raw `FLTEVT` register of `first_switch` (DESAT / OCP / TSD events).
    pub fltevt: u8,
    /// Raw `GFLTEVT` register (`0x29`) of `first_switch` (`bit0 = PRI_FLTN`, `bit1 = SEC_FLTN`; `0x03` = healthy).
    pub gfltevt: u8,
    /// Raw `SECUVEVT` register of `first_switch` (secondary supply UVLO events).
    pub secuvevt: u8,
}

#[cfg(feature = "board-jbldc")]
pub type DrvFaultStatus = EiceDriverFaultStatus;

#[cfg(feature = "board-jbldc")]
fn push_hex_u8(s: &mut String<128>, val: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let _ = s.push_str("0x");
    let _ = s.push(HEX[usize::from(val >> 4)] as char);
    let _ = s.push(HEX[usize::from(val & 0x0F)] as char);
}

/// F405 platform-specific faults
///
/// These faults can contain associated data from the hardware, such as
/// detailed DRV8301 fault status bits.
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub enum F405Fault {
    /// Over-current detected (from software or DRV8301)
    OverCurrent,
    /// Over-voltage on DC bus
    OverVoltage,
    /// Under-voltage on DC bus
    UnderVoltage,
    /// Over-temperature (FET)
    OverTemp,
    /// DRV8301 gate driver fault with detailed status
    DrvFault(DrvFaultStatus),
    /// DRV8301 SPI configuration failed at boot: the chip is running on its
    /// power-on defaults (shunt gain 10 instead of the calibrated 20, no
    /// VDS-OCP programmed), so every current reading and the software OC
    /// trip would be wrong. Kill class; the gate driver is held disabled.
    DrvConfigFailed,
    /// Hall sensor error (warning class: the ride continues on the
    /// fallback chain; the payload names the degradation, e.g. which wire)
    HallError(HallFaultKind),
    /// Command link stale while running (deadman / link-loss)
    CommTimeout,
    /// Graduated derating active (power rolloff > 20%) — warning class
    Derating,
}

impl PlatformFault for F405Fault {
    fn category(&self) -> FaultCategory {
        match self {
            Self::OverCurrent => FaultCategory::OverCurrent,
            Self::OverVoltage => FaultCategory::OverVoltage,
            Self::UnderVoltage => FaultCategory::UnderVoltage,
            Self::OverTemp => FaultCategory::OverTemp,
            Self::DrvFault(_) => FaultCategory::DriverFault,
            Self::DrvConfigFailed => FaultCategory::DriverFault,
            Self::HallError(_) => FaultCategory::HallError,
            Self::CommTimeout => FaultCategory::CommTimeout,
            Self::Derating => FaultCategory::Derating,
        }
    }

    fn details(&self) -> String<128> {
        let mut s = String::new();
        match self {
            #[cfg(not(feature = "board-jbldc"))]
            Self::DrvFault(status) => {
                // Build a string with all the DRV8301 fault flags
                if status.fetha_oc {
                    let _ = s.push_str("PhA_H_OC ");
                }
                if status.fetla_oc {
                    let _ = s.push_str("PhA_L_OC ");
                }
                if status.fethb_oc {
                    let _ = s.push_str("PhB_H_OC ");
                }
                if status.fetlb_oc {
                    let _ = s.push_str("PhB_L_OC ");
                }
                if status.fethc_oc {
                    let _ = s.push_str("PhC_H_OC ");
                }
                if status.fetlc_oc {
                    let _ = s.push_str("PhC_L_OC ");
                }
                if status.otsd {
                    let _ = s.push_str("ThermalShutdown ");
                }
                if status.otw {
                    let _ = s.push_str("ThermalWarn ");
                }
                if status.gvdd_uv {
                    let _ = s.push_str("GVDD_UV ");
                }
                if status.gvdd_ov {
                    let _ = s.push_str("GVDD_OV ");
                }
                if status.pvdd_uv {
                    let _ = s.push_str("PVDD_UV ");
                }
            }
            #[cfg(feature = "board-jbldc")]
            Self::DrvFault(status) => {
                const SW_NAMES: [&str; 6] = ["UH", "UL", "VH", "VL", "WH", "WL"];
                let sw = SW_NAMES
                    .get(usize::from(status.first_switch))
                    .copied()
                    .unwrap_or("GD");
                let _ = s.push_str(sw);
                if status.i2c_err_mask != 0 {
                    let _ = s.push_str(" I2C_ERR=");
                    push_hex_u8(&mut s, status.i2c_err_mask);
                }
                if status.unready_mask != 0 {
                    let _ = s.push_str(" UNRDY=");
                    push_hex_u8(&mut s, status.unready_mask);
                    let _ = s.push_str(" rdy=");
                    push_hex_u8(&mut s, status.rdy_stat);
                }
                if status.fltevt != 0 {
                    let _ = s.push_str(" flt=");
                    push_hex_u8(&mut s, status.fltevt);
                }
                if (status.gfltevt & 0x03) != 0x03 {
                    let _ = s.push_str(" gflt=");
                    push_hex_u8(&mut s, status.gfltevt);
                }
                if status.secuvevt != 0 {
                    let _ = s.push_str(" uv=");
                    push_hex_u8(&mut s, status.secuvevt);
                }
            }
            Self::DrvConfigFailed => {
                #[cfg(not(feature = "board-jbldc"))]
                let _ = s.push_str("config SPI failed, gate driver disabled");
                #[cfg(feature = "board-jbldc")]
                let _ = s.push_str("config I2C failed, gate driver disabled");
            }
            Self::HallError(kind) => {
                return kind.details();
            }
            _ => {
                // No additional details for other faults
            }
        }
        s
    }

    // severity(): central per-category policy (FaultCategory::severity).
    // OverTemp is GracefulStop, not Kill — "must not restart while hot"
    // survives via the any_stopping() start gate (no auto-clear on OT).

    fn from_hall_kind(kind: HallFaultKind) -> Option<Self> {
        Some(Self::HallError(kind))
    }

    /// Payload-free categories the shared core protection can raise.
    // DriverFault carries the DRV status payload — raised by the
    // platform DRV handler, not through this constructor.
    fn from_category(category: FaultCategory) -> Option<Self> {
        match category {
            FaultCategory::OverCurrent => Some(Self::OverCurrent),
            FaultCategory::OverVoltage => Some(Self::OverVoltage),
            FaultCategory::UnderVoltage => Some(Self::UnderVoltage),
            FaultCategory::OverTemp => Some(Self::OverTemp),
            FaultCategory::CommTimeout => Some(Self::CommTimeout),
            FaultCategory::Derating => Some(Self::Derating),
            _ => None,
        }
    }
}
