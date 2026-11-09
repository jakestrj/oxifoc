use crate::transport::{TransportConfig, TransportType};
use anyhow::{Context, Result};
use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Debug, Default, Deserialize, Clone)]
pub struct HostConfig {
    /// Transport type: "can" (default, aliases: "vcan", "socketcan")
    pub transport: Option<TransportType>,

    // CAN / vcan transport options
    pub can_interface: Option<String>, // e.g. "vcan0"

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

    pub fn transport_type(&self) -> TransportType {
        self.transport.clone().unwrap_or_default()
    }

    pub fn transport_config(&self) -> Result<TransportConfig> {
        match self.transport_type() {
            TransportType::Can => Ok(TransportConfig::Can {
                interface: self
                    .can_interface
                    .clone()
                    .unwrap_or_else(|| "vcan0".to_string()),
            }),
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
}
