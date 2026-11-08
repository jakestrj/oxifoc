// Build script: runs on the host at compile time — panicking IS the
// failure mode, so the firmware panic-policy lints don't apply here.
#![allow(clippy::unwrap_used, clippy::panic, clippy::string_slice)]

use std::fs;
use std::path::Path;

fn main() -> anyhow::Result<()> {
    println!("cargo:rerun-if-changed=proto");

    let proto_out = Path::new("src/proto");
    fs::create_dir_all(proto_out)?;

    let mut config = femtopb_build::Config::new();
    config
        .target(proto_out)
        .includes(&[Path::new("proto")])
        .protos(&[
            Path::new("proto/oxifoc/wire_message.proto"),
            Path::new("proto/common/options.proto"),
        ])
        .derive_defmt(false);
    config.compile()
}
