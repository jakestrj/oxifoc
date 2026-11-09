//! UDP transport for connecting to `oxifoc-virtual` (or CAN-over-UDP devices).

use std::sync::Arc;

use anyhow::{Context, Result};
use tokio::net::UdpSocket;
use tracing::info;

pub async fn connect(host: &str, port: u16) -> Result<Arc<UdpSocket>> {
    let addr = format!("{host}:{port}");
    info!("Connecting to UDP: {addr}");

    let socket = UdpSocket::bind("0.0.0.0:0")
        .await
        .context("Failed to bind UDP socket")?;
    socket
        .connect(&addr)
        .await
        .with_context(|| format!("Failed to connect UDP to {addr}"))?;

    info!("UDP connected to {addr}");
    Ok(Arc::new(socket))
}
