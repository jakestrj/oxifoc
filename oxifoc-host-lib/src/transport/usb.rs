//! USB transport stub (legacy ergot USB transport replaced by CAN / streaming-protocol).

use anyhow::{Result, bail};

pub async fn connect() -> Result<()> {
    bail!("USB bulk transport is not supported in CAN streaming-protocol mode; use --transport vcan, tcp, or udp")
}
