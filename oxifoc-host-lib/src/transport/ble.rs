//! BLE NUS transport for oxifoc host communication.

use anyhow::{Context, Result, anyhow};
use bluest::{Adapter, Characteristic, Device, Uuid};
use tracing::info;

// NUS UUIDs
const NUS_SERVICE_UUID: Uuid = Uuid::from_u128(0x6e400001_b5a3_f393_e0a9_e50e24dcca9e);
const NUS_RX_UUID: Uuid = Uuid::from_u128(0x6e400002_b5a3_f393_e0a9_e50e24dcca9e);
const NUS_TX_UUID: Uuid = Uuid::from_u128(0x6e400003_b5a3_f393_e0a9_e50e24dcca9e);

pub struct BleConnection {
    pub rx_char: Characteristic,
    pub tx_char: Characteristic,
}

pub async fn connect(device: &Device) -> Result<BleConnection> {
    let adapter = Adapter::default()
        .await
        .context("Failed to get BLE adapter")?;
    info!("Connecting to BLE device...");
    adapter
        .connect_device(device)
        .await
        .context("BLE connection failed")?;
    info!("BLE connected");

    let services = device
        .discover_services_with_uuid(NUS_SERVICE_UUID)
        .await
        .context("NUS service discovery failed")?;

    let nus_service = services
        .first()
        .ok_or_else(|| anyhow!("NUS service not found on device"))?;

    let chars = nus_service
        .discover_characteristics()
        .await
        .context("Failed to discover NUS characteristics")?;

    let rx_char = chars
        .iter()
        .find(|c| c.uuid() == NUS_RX_UUID)
        .ok_or_else(|| anyhow!("NUS RX characteristic not found"))?
        .clone();

    let tx_char = chars
        .iter()
        .find(|c| c.uuid() == NUS_TX_UUID)
        .ok_or_else(|| anyhow!("NUS TX characteristic not found"))?
        .clone();

    Ok(BleConnection {
        rx_char,
        tx_char,
    })
}
