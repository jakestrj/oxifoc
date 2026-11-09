use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use protox::prost::Message as _;

fn main() {
    let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let workspace_root = manifest.parent().unwrap();

    // Compile the protobuf FileDescriptorSet for `oxifoc.WireMessage` (including
    // transitive imports) so MCAP recordings embed the full schema definition.
    let proto_dir = workspace_root.join("oxifoc-core").join("proto");
    println!("cargo:rerun-if-changed={}", proto_dir.display());
    let fds = protox::compile(["oxifoc/wire_message.proto"], [&proto_dir])
        .expect("Failed to compile oxifoc/wire_message.proto descriptor set");
    let out_dir = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    fs::write(
        out_dir.join("wire_message_descriptor.bin"),
        fds.encode_to_vec(),
    )
    .expect("Failed to write wire_message_descriptor.bin");

    let plot_ui = workspace_root.join("slint-wgpu-plot").join("ui");

    let mut library_paths = HashMap::new();
    library_paths.insert("slint-wgpu-plot".to_string(), plot_ui);

    slint_build::compile_with_config(
        "ui/app.slint",
        slint_build::CompilerConfiguration::new().with_library_paths(library_paths),
    )
    .unwrap();
}
