//! PEAK-System PCAN-USB transport backend via [`pcbusb`].
//!
//! On macOS, `pcbusb` links against MacCAN's `libPCBUSB.dylib` (`/usr/local/lib/libPCBUSB.dylib`).
//! Elsewhere, `pcbusb` binds to PEAK's `PCAN-Basic` library.
//!
//! Because upstream `pcbusb` (`v0.1.5`) opens `PCAN_USBBUS1` only and implements a busy-spin in
//! `embedded_can::blocking::Can::receive`, this adapter wraps `pcbusb::Interface` in an
//! `Arc<Mutex<_>>` and drives non-blocking `embedded_can::nb::Can` reads from a Tokio poll loop.

use anyhow::{Result, bail};

#[cfg(feature = "pcan")]
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

/// Default interface alias for the primary PCAN-USB channel (`PCAN_USBBUS1`).
pub const DEFAULT_PCAN_INTERFACE: &str = "pcan0";

/// Whether `oxifoc-host-lib` was compiled with the `pcan` Cargo feature enabled.
pub const PCAN_FEATURE_ENABLED: bool = cfg!(feature = "pcan");

/// Poll cadence while the `libPCBUSB` receive FIFO reports `PCAN_ERROR_QRCVEMPTY`.
/// Queued bursts are drained back-to-back with zero sleep.
#[cfg(feature = "pcan")]
const PCAN_RX_POLL_INTERVAL: Duration = Duration::from_micros(500);

/// Backoff delay when the adapter's transmit queue momentarily reports full.
#[cfg(feature = "pcan")]
const PCAN_TX_RETRY_DELAY: Duration = Duration::from_micros(500);

/// Maximum transmit retries before surfacing a full-queue error.
#[cfg(feature = "pcan")]
const PCAN_TX_MAX_RETRIES: u32 = 8;

/// Returns `true` when `can_interface` names a PEAK PCAN-USB interface rather than a
/// SocketCAN or UDP Multicast Virtual CAN (`vcan`) bus.
#[must_use]
pub fn is_pcan_interface(can_interface: &str) -> bool {
    let trimmed = can_interface.trim();
    if trimmed.is_empty() || trimmed.starts_with("mcast:") {
        return false;
    }
    let lower = trimmed.to_ascii_lowercase();
    lower.starts_with("pcan") || lower == "pcbusb"
}

/// Validate that `can_interface` maps to the single channel supported by upstream `pcbusb`
/// (`PCAN_USBBUS1`), returning the canonical PCAN-Basic channel name.
pub fn validate_pcan_channel(can_interface: &str) -> Result<&'static str> {
    let trimmed = can_interface.trim();
    let lower = trimmed.to_ascii_lowercase();
    match lower.as_str() {
        "" | "pcan" | "pcan0" | "pcan:0" | "pcan:usb1" | "pcan_usbbus1" | "pcan-usbbus1"
        | "usb1" | "usbbus1" | "pcbusb" => Ok("PCAN_USBBUS1"),
        _ => bail!(
            "unsupported PCAN interface '{trimmed}': upstream pcbusb supports only channel 1 \
             ('pcan0' / 'PCAN_USBBUS1')"
        ),
    }
}

/// Validate that `bitrate` (in bit/s) is supported by PCAN-Basic / MacCAN `PCBUSB`.
pub fn validate_pcan_bitrate(bitrate: u32) -> Result<()> {
    if !matches!(
        bitrate,
        1_000_000
            | 800_000
            | 666_666
            | 666_667
            | 500_000
            | 250_000
            | 125_000
            | 100_000
            | 95_000
            | 95_238
            | 83_000
            | 83_333
            | 50_000
            | 47_000
            | 47_619
            | 33_000
            | 33_333
            | 20_000
            | 10_000
            | 5_000
    ) {
        bail!(
            "unsupported PCAN bitrate {bitrate} bps \
             (supported: 1000000, 800000, 500000, 250000, 125000, 100000, 50000, 20000, 10000, 5000)"
        );
    }
    Ok(())
}

/// Check that the `pcan` Cargo feature is compiled in, returning an actionable error otherwise.
pub fn ensure_pcan_feature_enabled() -> Result<()> {
    if !PCAN_FEATURE_ENABLED {
        bail!(
            "PCAN-USB support is not compiled in: rebuild with '--features pcan' \
             (requires MacCAN libPCBUSB in /usr/local/lib on macOS)"
        );
    }
    Ok(())
}

#[cfg(feature = "pcan")]
fn baudrate_from_bps(bitrate: u32) -> Result<pcbusb::Baudrate> {
    use pcbusb::Baudrate;
    validate_pcan_bitrate(bitrate)?;
    Ok(match bitrate {
        1_000_000 => Baudrate::Baud1m,
        800_000 => Baudrate::Baud800k,
        666_666 | 666_667 => Baudrate::Baud666k,
        500_000 => Baudrate::Baud500k,
        250_000 => Baudrate::Baud250k,
        125_000 => Baudrate::Baud125k,
        100_000 => Baudrate::Baud100k,
        95_000 | 95_238 => Baudrate::Baud95k,
        83_000 | 83_333 => Baudrate::Baud83k,
        50_000 => Baudrate::Baud50k,
        47_000 | 47_619 => Baudrate::Baud47k,
        33_000 | 33_333 => Baudrate::Baud33k,
        20_000 => Baudrate::Baud20k,
        10_000 => Baudrate::Baud10k,
        5_000 => Baudrate::Baud5k,
        _ => unreachable!("validated by validate_pcan_bitrate"),
    })
}

/// Thread-safe handle to an initialized `PCAN_USBBUS1` adapter.
///
/// Dropping the last clone drops [`pcbusb::Interface`], which invokes `CAN_Uninitialize`.
#[cfg(feature = "pcan")]
#[derive(Clone)]
pub struct PcanEndpoint {
    iface: Arc<Mutex<pcbusb::Interface>>,
}

#[cfg(feature = "pcan")]
impl PcanEndpoint {
    /// Open and initialize the PCAN-USB adapter at `bitrate` bps.
    pub fn open(can_interface: &str, bitrate: u32) -> Result<Self> {
        use anyhow::Context as _;
        use tracing::info;

        let channel_name = validate_pcan_channel(can_interface)?;
        let baudrate = baudrate_from_bps(bitrate)?;

        let mut iface = pcbusb::Interface::init(baudrate).with_context(|| {
            format!(
                "initialize PCAN-USB {channel_name} ('{can_interface}') at {bitrate} bps \
                 (verify adapter is connected and not in use)"
            )
        })?;

        // Ensure `PCAN_FILTER_OPEN` is active; software arbitration-ID filtering (`0x7ED`)
        // runs in `backend_main`. Avoid `Filter::new(...)` on `pcbusb` v0.1.5, whose 11-bit
        // SJA1000 mask/code layout is inverted.
        iface
            .add_filter(&pcbusb::Filter::accept_all())
            .context("open PCAN-USB message acceptance filter")?;

        info!("Opened PCAN-USB interface '{channel_name}' ('{can_interface}') at {bitrate} bps");
        Ok(Self {
            iface: Arc::new(Mutex::new(iface)),
        })
    }

    /// Transmit a single 8-byte standard 11-bit CAN data frame.
    pub async fn send_frame(&self, arb_id: u16, payload: &[u8; 8]) -> Result<()> {
        use anyhow::Context as _;
        use pcbusb::prelude::*;

        let std_id = pcbusb::StandardId::new(arb_id)
            .ok_or_else(|| anyhow::anyhow!("invalid 11-bit CAN arbitration ID 0x{arb_id:03X}"))?;
        let frame = pcbusb::Frame::new(std_id, payload)
            .ok_or_else(|| anyhow::anyhow!("invalid 8-byte PCAN frame payload"))?;

        for attempt in 0..=PCAN_TX_MAX_RETRIES {
            let tx_res = {
                let mut guard = self
                    .iface
                    .lock()
                    .map_err(|_| anyhow::anyhow!("PCAN-USB mutex poisoned"))?;
                guard.transmit(&frame)
            };
            match tx_res {
                Ok(_) => return Ok(()),
                Err(::nb::Error::WouldBlock) if attempt < PCAN_TX_MAX_RETRIES => {
                    tokio::time::sleep(PCAN_TX_RETRY_DELAY).await;
                }
                Err(::nb::Error::WouldBlock) => {
                    bail!(
                        "PCAN-USB transmit queue remained full after {PCAN_TX_MAX_RETRIES} retries"
                    );
                }
                Err(::nb::Error::Other(err)) => {
                    return Err(anyhow::Error::new(err)).context("transmit PCAN-USB frame");
                }
            }
        }
        Ok(())
    }

    /// Poll the adapter's non-blocking receive FIFO until an 8-byte standard 11-bit data frame
    /// arrives. Queued bursts drain immediately with no sleep between frames.
    pub async fn recv_frame(&self) -> Result<(u16, [u8; 8])> {
        use anyhow::Context as _;
        use pcbusb::prelude::*;

        loop {
            let rx_res = {
                let mut guard = self
                    .iface
                    .lock()
                    .map_err(|_| anyhow::anyhow!("PCAN-USB mutex poisoned"))?;
                guard.receive()
            };
            match rx_res {
                Ok(frame) => {
                    if !frame.is_extended()
                        && !frame.is_remote_frame()
                        && let Ok(data) = <[u8; 8]>::try_from(frame.data())
                        && let pcbusb::Id::Standard(std_id) = frame.id()
                    {
                        return Ok((std_id.as_raw(), data));
                    }
                }
                Err(::nb::Error::WouldBlock) => {
                    tokio::time::sleep(PCAN_RX_POLL_INTERVAL).await;
                }
                Err(::nb::Error::Other(err)) => {
                    return Err(anyhow::Error::new(err)).context("read PCAN-USB frame");
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pcan_interface_detection_distinguishes_virtual_and_hardware_names() {
        assert!(is_pcan_interface("pcan"));
        assert!(is_pcan_interface("pcan0"));
        assert!(is_pcan_interface("PCAN_USBBUS1"));
        assert!(is_pcan_interface("pcan:usb1"));
        assert!(is_pcan_interface("pcbusb"));

        assert!(!is_pcan_interface(""));
        assert!(!is_pcan_interface("vcan0"));
        assert!(!is_pcan_interface("can0"));
        assert!(!is_pcan_interface("mcast:pcan0"));
        assert!(!is_pcan_interface("239.255.79.88:29536"));
    }

    #[test]
    fn pcan_channel_validation_accepts_usbbus1_aliases_and_rejects_secondary_channels() {
        for valid in [
            "",
            "pcan",
            "pcan0",
            "pcan:0",
            "pcan:usb1",
            "PCAN_USBBUS1",
            "usb1",
            "pcbusb",
        ] {
            assert_eq!(validate_pcan_channel(valid).unwrap(), "PCAN_USBBUS1");
        }

        for unsupported in ["pcan1", "PCAN_USBBUS2", "usb2", "vcan0"] {
            let err = validate_pcan_channel(unsupported).unwrap_err();
            assert!(err.to_string().contains("PCAN_USBBUS1"));
        }
    }

    #[test]
    fn pcan_bitrate_validation_covers_standard_rates() {
        for rate in [
            1_000_000, 800_000, 500_000, 250_000, 125_000, 100_000, 50_000,
        ] {
            validate_pcan_bitrate(rate).expect("standard CAN bitrate must be accepted");
        }
        assert!(validate_pcan_bitrate(0).is_err());
        assert!(validate_pcan_bitrate(123_456).is_err());
    }
}
