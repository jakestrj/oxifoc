use crate::transport::{TransportConfig, TransportType, pcan};
use anyhow::{Context, Result};
use oxifoc_core::icd::CAN_BITRATE;
use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Debug, Default, Deserialize, Clone)]
pub struct HostConfig {
    /// Transport type: "can" (default, aliases: "vcan", "socketcan") or "pcan" (alias: "pcbusb")
    pub transport: Option<TransportType>,

    /// CAN / Virtual CAN / PCAN interface name (e.g. "vcan0", "can0", "pcan0")
    pub can_interface: Option<String>,

    /// CAN bus bitrate in bits per second (default: `CAN_BITRATE` = 500_000).
    /// Used when opening user-space USB CAN adapters such as PCAN-USB on macOS.
    pub can_bitrate: Option<u32>,

    /// Reconnection policy. None = use default (infinite retries)
    pub reconnect: Option<ReconnectPolicy>,

    /// Desired fast telemetry rate in Hz (default 0 = disabled).
    pub fast_hz: Option<u16>,
}

/// Controls how the host handles transport disconnection/failure.
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ReconnectPolicy {
    /// Disconnect immediately on any failure, including loss of an established
    /// connection, with no retry.
    None,
    /// Allow at most N consecutive connection generations before giving up.
    /// A completed protocol handshake resets the failure count.
    Limited(u32),
    /// Retry forever (default behavior)
    Infinite,
}

impl HostConfig {
    pub fn load_default() -> Result<Option<Self>> {
        if let Some(p) = env::var_os("OXIFOC_HOST_CONFIG") {
            return Self::from_path(PathBuf::from(p)).map(Some);
        }
        let cwd = env::current_dir().context("resolve current directory for host config")?;
        let p = cwd.join("oxifoc-host.toml");
        if p.exists() {
            return Self::from_path(p).map(Some);
        }
        Ok(None)
    }

    fn from_path(path: PathBuf) -> Result<Self> {
        let contents = fs::read_to_string(&path)
            .with_context(|| format!("read host config {}", path.display()))?;
        toml::from_str::<Self>(&contents)
            .with_context(|| format!("parse host config {} as TOML", path.display()))
    }

    pub fn reconnect_policy(&self) -> ReconnectPolicy {
        self.reconnect.unwrap_or(ReconnectPolicy::Infinite)
    }

    pub fn fast_hz(&self) -> u16 {
        self.fast_hz.unwrap_or(0)
    }

    pub fn can_bitrate(&self) -> u32 {
        self.can_bitrate.unwrap_or(CAN_BITRATE)
    }

    pub fn transport_type(&self) -> TransportType {
        self.transport.clone().unwrap_or_default()
    }

    pub fn transport_config(&self) -> Result<TransportConfig> {
        let bitrate = self.can_bitrate();
        match self.transport_type() {
            TransportType::Can => {
                let interface = self
                    .can_interface
                    .clone()
                    .unwrap_or_else(|| "vcan0".to_string());
                if pcan::is_pcan_interface(&interface) {
                    pcan::validate_pcan_channel(&interface)?;
                    pcan::validate_pcan_bitrate(bitrate)?;
                    pcan::ensure_pcan_feature_enabled()?;
                    return Ok(TransportConfig::Pcan { interface, bitrate });
                }
                Ok(TransportConfig::Can { interface, bitrate })
            }
            TransportType::Pcan => {
                let interface = self
                    .can_interface
                    .clone()
                    .unwrap_or_else(|| pcan::DEFAULT_PCAN_INTERFACE.to_string());
                pcan::validate_pcan_channel(&interface)?;
                pcan::validate_pcan_bitrate(bitrate)?;
                pcan::ensure_pcan_feature_enabled()?;
                Ok(TransportConfig::Pcan { interface, bitrate })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_config_path(name: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock before unix epoch")
            .as_nanos();
        env::temp_dir().join(format!("oxifoc-host-{name}-{nonce}.toml"))
    }

    #[test]
    fn explicit_invalid_config_is_an_error() {
        let path = temp_config_path("invalid");
        fs::write(&path, "transport = [not valid toml").expect("write temporary invalid config");

        let err = HostConfig::from_path(path.clone()).expect_err("invalid config must fail");
        assert!(err.to_string().contains("parse host config"));

        fs::remove_file(path).expect("remove temporary config");
    }

    #[test]
    fn missing_explicit_config_is_an_error() {
        let path = temp_config_path("missing");
        let err = HostConfig::from_path(path).expect_err("missing config must fail");
        assert!(err.to_string().contains("read host config"));
    }

    #[test]
    fn pcan_toml_deserializes_with_default_and_custom_bitrate() {
        let parsed: HostConfig = toml::from_str(
            r#"
            transport = "pcan"
            can_interface = "pcan0"
            can_bitrate = 1000000
            "#,
        )
        .expect("parse pcan config");
        assert_eq!(parsed.transport_type(), TransportType::Pcan);
        assert_eq!(parsed.can_bitrate(), 1_000_000);

        #[cfg(feature = "pcan")]
        assert_eq!(
            parsed.transport_config().unwrap(),
            TransportConfig::Pcan {
                interface: "pcan0".to_string(),
                bitrate: 1_000_000,
            }
        );

        #[cfg(not(feature = "pcan"))]
        assert!(
            parsed
                .transport_config()
                .unwrap_err()
                .to_string()
                .contains("--features pcan")
        );
    }
}
