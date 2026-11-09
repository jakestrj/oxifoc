//! MCAP v0 wire-message logger for `oxifoc.WireMessage`.
//!
//! Writes standard MCAP v0 files containing a single protobuf channel
//! (`/oxifoc/wire_message`, schema `oxifoc.WireMessage` with the embedded
//! `google.protobuf.FileDescriptorSet`) and full Summary + SummaryOffset + CRC
//! records so files open directly in Foxglove Studio and `mcap` CLI tools.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use chrono::Local;
use slint::{ComponentHandle, SharedString};

use super::App;

/// Serialized `google.protobuf.FileDescriptorSet` for `oxifoc/wire_message.proto`
/// and its transitive imports (`common/options.proto`, `google/protobuf/descriptor.proto`).
pub const WIRE_MESSAGE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/wire_message_descriptor.bin"));

pub const WIRE_MESSAGE_SCHEMA_NAME: &str = "oxifoc.WireMessage";
pub const WIRE_MESSAGE_TOPIC: &str = "/oxifoc/wire_message";
pub const PROTOBUF_ENCODING: &str = "protobuf";

const MCAP_MAGIC: &[u8; 8] = b"\x89MCAP0\r\n";

const OP_HEADER: u8 = 0x01;
const OP_FOOTER: u8 = 0x02;
const OP_SCHEMA: u8 = 0x03;
const OP_CHANNEL: u8 = 0x04;
const OP_MESSAGE: u8 = 0x05;
const OP_STATISTICS: u8 = 0x0B;
const OP_SUMMARY_OFFSET: u8 = 0x0E;
const OP_DATA_END: u8 = 0x0F;

const SCHEMA_ID: u16 = 1;
const CHANNEL_ID: u16 = 1;

pub(super) type SharedMcapRecorder = Arc<Mutex<Option<McapFileRecorder>>>;

struct CountingCrcWriter<W> {
    inner: W,
    bytes_written: u64,
    hasher: crc32fast::Hasher,
}

impl<W: Write> CountingCrcWriter<W> {
    fn new(inner: W) -> Self {
        Self {
            inner,
            bytes_written: 0,
            hasher: crc32fast::Hasher::new(),
        }
    }

    fn write_bytes(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.inner.write_all(bytes)?;
        self.bytes_written += bytes.len() as u64;
        self.hasher.update(bytes);
        Ok(())
    }

    fn write_record(&mut self, op: u8, payload: &[u8]) -> io::Result<()> {
        self.write_bytes(&[op])?;
        self.write_bytes(&(payload.len() as u64).to_le_bytes())?;
        self.write_bytes(payload)
    }

    fn current_crc(&self) -> u32 {
        self.hasher.clone().finalize()
    }

    fn reset_crc(&mut self) {
        self.hasher = crc32fast::Hasher::new();
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

fn push_prefixed_str(buf: &mut Vec<u8>, s: &str) {
    buf.extend_from_slice(&(s.len() as u32).to_le_bytes());
    buf.extend_from_slice(s.as_bytes());
}

fn push_prefixed_bytes(buf: &mut Vec<u8>, data: &[u8]) {
    buf.extend_from_slice(&(data.len() as u32).to_le_bytes());
    buf.extend_from_slice(data);
}

fn encode_header_payload(profile: &str, library: &str) -> Vec<u8> {
    let mut buf = Vec::with_capacity(8 + profile.len() + library.len());
    push_prefixed_str(&mut buf, profile);
    push_prefixed_str(&mut buf, library);
    buf
}

fn encode_schema_payload(id: u16, name: &str, encoding: &str, data: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(2 + 4 + name.len() + 4 + encoding.len() + 4 + data.len());
    buf.extend_from_slice(&id.to_le_bytes());
    push_prefixed_str(&mut buf, name);
    push_prefixed_str(&mut buf, encoding);
    push_prefixed_bytes(&mut buf, data);
    buf
}

fn encode_channel_payload(id: u16, schema_id: u16, topic: &str, message_encoding: &str) -> Vec<u8> {
    let mut buf = Vec::with_capacity(4 + 4 + topic.len() + 4 + message_encoding.len() + 4);
    buf.extend_from_slice(&id.to_le_bytes());
    buf.extend_from_slice(&schema_id.to_le_bytes());
    push_prefixed_str(&mut buf, topic);
    push_prefixed_str(&mut buf, message_encoding);
    // Empty metadata Map<string, string> (0 byte length)
    buf.extend_from_slice(&0u32.to_le_bytes());
    buf
}

/// Streaming MCAP v0 writer for `oxifoc.WireMessage` payloads.
pub struct McapWriter<W: Write> {
    writer: CountingCrcWriter<W>,
    sequence: u32,
    message_count: u64,
    message_start_time: Option<u64>,
    message_end_time: Option<u64>,
    finished: bool,
}

impl<W: Write> McapWriter<W> {
    /// Initialize a new MCAP stream and write the leading magic, `Header`,
    /// `Schema` (`oxifoc.WireMessage`), and `Channel` (`/oxifoc/wire_message`).
    pub fn new(inner: W) -> io::Result<Self> {
        let mut writer = CountingCrcWriter::new(inner);
        writer.write_bytes(MCAP_MAGIC)?;

        let header =
            encode_header_payload("", concat!("oxifoc-host-slint ", env!("CARGO_PKG_VERSION")));
        writer.write_record(OP_HEADER, &header)?;

        let schema = encode_schema_payload(
            SCHEMA_ID,
            WIRE_MESSAGE_SCHEMA_NAME,
            PROTOBUF_ENCODING,
            WIRE_MESSAGE_DESCRIPTOR_SET,
        );
        writer.write_record(OP_SCHEMA, &schema)?;

        let channel =
            encode_channel_payload(CHANNEL_ID, SCHEMA_ID, WIRE_MESSAGE_TOPIC, PROTOBUF_ENCODING);
        writer.write_record(OP_CHANNEL, &channel)?;

        Ok(Self {
            writer,
            sequence: 0,
            message_count: 0,
            message_start_time: None,
            message_end_time: None,
            finished: false,
        })
    }

    /// Write a single protobuf-encoded `oxifoc.WireMessage` with nanosecond UNIX timestamp.
    pub fn write_message(&mut self, timestamp_ns: u64, data: &[u8]) -> io::Result<()> {
        let record_len = 22u64 + data.len() as u64;
        self.writer.write_bytes(&[OP_MESSAGE])?;
        self.writer.write_bytes(&record_len.to_le_bytes())?;
        self.writer.write_bytes(&CHANNEL_ID.to_le_bytes())?;
        self.writer.write_bytes(&self.sequence.to_le_bytes())?;
        self.writer.write_bytes(&timestamp_ns.to_le_bytes())?;
        self.writer.write_bytes(&timestamp_ns.to_le_bytes())?;
        self.writer.write_bytes(data)?;

        self.sequence = self.sequence.wrapping_add(1);
        self.message_count += 1;
        self.message_start_time = Some(
            self.message_start_time
                .map_or(timestamp_ns, |t| t.min(timestamp_ns)),
        );
        self.message_end_time = Some(
            self.message_end_time
                .map_or(timestamp_ns, |t| t.max(timestamp_ns)),
        );
        Ok(())
    }

    /// Total number of `WireMessage` records written so far.
    #[must_use]
    pub fn message_count(&self) -> u64 {
        self.message_count
    }

    fn finish_inner(&mut self) -> io::Result<()> {
        if self.finished {
            return Ok(());
        }
        self.finished = true;

        // 1. DataEnd record with CRC32 of the entire Data section (from file start).
        let data_section_crc = self.writer.current_crc();
        self.writer
            .write_record(OP_DATA_END, &data_section_crc.to_le_bytes())?;

        // 2. Summary section (CRC32 reset at summary_start).
        self.writer.reset_crc();
        let summary_start = self.writer.bytes_written;

        let schema_group_start = self.writer.bytes_written;
        let schema = encode_schema_payload(
            SCHEMA_ID,
            WIRE_MESSAGE_SCHEMA_NAME,
            PROTOBUF_ENCODING,
            WIRE_MESSAGE_DESCRIPTOR_SET,
        );
        self.writer.write_record(OP_SCHEMA, &schema)?;
        let schema_group_len = self.writer.bytes_written - schema_group_start;

        let channel_group_start = self.writer.bytes_written;
        let channel =
            encode_channel_payload(CHANNEL_ID, SCHEMA_ID, WIRE_MESSAGE_TOPIC, PROTOBUF_ENCODING);
        self.writer.write_record(OP_CHANNEL, &channel)?;
        let channel_group_len = self.writer.bytes_written - channel_group_start;

        let stats_group_start = self.writer.bytes_written;
        let mut stats = Vec::with_capacity(48);
        stats.extend_from_slice(&self.message_count.to_le_bytes()); // message_count: u64
        stats.extend_from_slice(&1u16.to_le_bytes()); // schema_count: u16
        stats.extend_from_slice(&1u32.to_le_bytes()); // channel_count: u32
        stats.extend_from_slice(&0u32.to_le_bytes()); // attachment_count: u32
        stats.extend_from_slice(&0u32.to_le_bytes()); // metadata_count: u32
        stats.extend_from_slice(&0u32.to_le_bytes()); // chunk_count: u32
        stats.extend_from_slice(&self.message_start_time.unwrap_or(0).to_le_bytes());
        stats.extend_from_slice(&self.message_end_time.unwrap_or(0).to_le_bytes());
        if self.message_count > 0 {
            stats.extend_from_slice(&10u32.to_le_bytes()); // Map<u16, u64> byte length = 2 + 8
            stats.extend_from_slice(&CHANNEL_ID.to_le_bytes());
            stats.extend_from_slice(&self.message_count.to_le_bytes());
        } else {
            stats.extend_from_slice(&0u32.to_le_bytes());
        }
        self.writer.write_record(OP_STATISTICS, &stats)?;
        let stats_group_len = self.writer.bytes_written - stats_group_start;

        // 3. SummaryOffset section.
        let summary_offset_start = self.writer.bytes_written;
        for (group_op, group_start, group_len) in [
            (OP_SCHEMA, schema_group_start, schema_group_len),
            (OP_CHANNEL, channel_group_start, channel_group_len),
            (OP_STATISTICS, stats_group_start, stats_group_len),
        ] {
            let mut so = [0u8; 17];
            so[0] = group_op;
            so[1..9].copy_from_slice(&group_start.to_le_bytes());
            so[9..17].copy_from_slice(&group_len.to_le_bytes());
            self.writer.write_record(OP_SUMMARY_OFFSET, &so)?;
        }

        // 4. Footer record: summary_crc covers summary_start through summary_offset_start field.
        self.writer.write_bytes(&[OP_FOOTER])?;
        self.writer.write_bytes(&20u64.to_le_bytes())?;
        self.writer.write_bytes(&summary_start.to_le_bytes())?;
        self.writer
            .write_bytes(&summary_offset_start.to_le_bytes())?;
        let summary_crc = self.writer.current_crc();
        self.writer.write_bytes(&summary_crc.to_le_bytes())?;

        // 5. Trailing magic.
        self.writer.write_bytes(MCAP_MAGIC)?;
        self.writer.flush()
    }

    /// Finalize the MCAP file and flush the underlying writer.
    pub fn finish(&mut self) -> io::Result<()> {
        self.finish_inner()
    }
}

impl<W: Write> Drop for McapWriter<W> {
    fn drop(&mut self) {
        let _ = self.finish_inner();
    }
}

/// File-backed MCAP recorder with its target path.
pub struct McapFileRecorder {
    path: PathBuf,
    writer: McapWriter<BufWriter<File>>,
}

impl McapFileRecorder {
    pub fn create(path: impl AsRef<Path>) -> io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        let file = File::create(&path)?;
        let writer = McapWriter::new(BufWriter::new(file))?;
        Ok(Self { path, writer })
    }

    pub fn create_timestamped_in_logs() -> io::Result<Self> {
        let logs_dir = Path::new("logs");
        std::fs::create_dir_all(logs_dir)?;
        let base = Local::now().format("oxifoc_%Y%m%d_%H%M%S").to_string();
        let mut candidate = logs_dir.join(format!("{base}.mcap"));
        let mut suffix = 1u32;
        while candidate.exists() {
            candidate = logs_dir.join(format!("{base}_{suffix}.mcap"));
            suffix += 1;
        }
        Self::create(candidate)
    }

    pub fn write_message(&mut self, timestamp_ns: u64, data: &[u8]) -> io::Result<()> {
        self.writer.write_message(timestamp_ns, data)
    }

    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    #[must_use]
    pub fn message_count(&self) -> u64 {
        self.writer.message_count()
    }

    pub fn finish(mut self) -> io::Result<(PathBuf, u64)> {
        let count = self.writer.message_count();
        self.writer.finish_inner()?;
        Ok((self.path, count))
    }
}

pub(super) fn stop_recording(recorder: &SharedMcapRecorder, app: Option<&App>) {
    if let Some(rec) = recorder.lock().unwrap().take() {
        match rec.finish() {
            Ok((path, count)) => {
                let path_str = path.display().to_string();
                tracing::info!("MCAP recording saved: {path_str} ({count} messages)");
                if let Some(app) = app {
                    app.set_mcap_recording(false);
                    app.set_mcap_status(SharedString::from(format!("{path_str} ({count} msgs)")));
                }
            }
            Err(e) => {
                tracing::error!("Failed to finalize MCAP recording: {e}");
                if let Some(app) = app {
                    app.set_mcap_recording(false);
                    app.set_mcap_status(SharedString::from(format!("MCAP error: {e}")));
                }
            }
        }
    } else if let Some(app) = app {
        app.set_mcap_recording(false);
    }
}

pub(super) fn bind(app: &App, recorder: &SharedMcapRecorder) {
    {
        let weak = app.as_weak();
        let rec = recorder.clone();
        app.on_mcap_start(move || {
            let Some(app) = weak.upgrade() else { return };
            stop_recording(&rec, None);
            match McapFileRecorder::create_timestamped_in_logs() {
                Ok(new_rec) => {
                    let path_str = new_rec.path().display().to_string();
                    tracing::info!("MCAP recording started: {path_str}");
                    app.set_mcap_recording(true);
                    app.set_mcap_status(SharedString::from(path_str));
                    *rec.lock().unwrap() = Some(new_rec);
                }
                Err(e) => {
                    tracing::error!("Failed to start MCAP recording: {e}");
                    app.set_mcap_recording(false);
                    app.set_mcap_status(SharedString::from(format!("MCAP error: {e}")));
                }
            }
        });
    }

    {
        let weak = app.as_weak();
        let rec = recorder.clone();
        app.on_mcap_stop(move || {
            let app = weak.upgrade();
            stop_recording(&rec, app.as_ref());
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use femtopb::UnknownFields;
    use oxifoc_core::proto::oxifoc::{TelemetryConfigRequest, WireMessage, wire_message::Payload};
    use oxifoc_host_lib::encode_wire_message_proto;

    #[test]
    fn descriptor_set_is_embedded_and_non_empty() {
        assert!(!WIRE_MESSAGE_DESCRIPTOR_SET.is_empty());
    }

    #[test]
    fn mcap_writer_produces_valid_records_and_crcs() {
        let mut buf = Vec::new();
        {
            let mut writer = McapWriter::new(&mut buf).unwrap();
            let msg = WireMessage {
                payload: Some(Payload::TelemetryConfig(TelemetryConfigRequest {
                    fast_hz: 1000,
                    unknown_fields: UnknownFields::empty(),
                })),
                unknown_fields: UnknownFields::empty(),
            };
            let encoded = encode_wire_message_proto(&msg);

            writer
                .write_message(1_700_000_000_000_000_000, &encoded)
                .unwrap();
            writer
                .write_message(1_700_000_000_001_000_000, &encoded)
                .unwrap();
            assert_eq!(writer.message_count(), 2);
        }

        // Verify leading and trailing MCAP magic bytes
        assert!(buf.starts_with(MCAP_MAGIC));
        assert!(buf.ends_with(MCAP_MAGIC));

        // Walk all records between leading and trailing magic
        let mut pos = MCAP_MAGIC.len();
        let end = buf.len() - MCAP_MAGIC.len();
        let mut ops = Vec::new();
        let mut data_end_offset = None;
        let mut footer_offset = None;

        while pos < end {
            let op = buf[pos];
            let rec_len = u64::from_le_bytes(buf[pos + 1..pos + 9].try_into().unwrap()) as usize;
            if op == OP_DATA_END {
                data_end_offset = Some(pos);
                let expected_crc = crc32fast::hash(&buf[..pos]);
                let actual_crc = u32::from_le_bytes(buf[pos + 9..pos + 13].try_into().unwrap());
                assert_eq!(actual_crc, expected_crc, "DataEnd CRC mismatch");
            }
            if op == OP_FOOTER {
                footer_offset = Some(pos);
                assert_eq!(rec_len, 20);
                let summary_start =
                    u64::from_le_bytes(buf[pos + 9..pos + 17].try_into().unwrap()) as usize;
                let expected_summary_crc = crc32fast::hash(&buf[summary_start..pos + 25]);
                let actual_summary_crc =
                    u32::from_le_bytes(buf[pos + 25..pos + 29].try_into().unwrap());
                assert_eq!(
                    actual_summary_crc, expected_summary_crc,
                    "Footer summary CRC mismatch"
                );
            }
            ops.push(op);
            pos += 9 + rec_len;
        }

        assert_eq!(pos, end);
        assert!(data_end_offset.is_some());
        assert!(footer_offset.is_some());
        assert_eq!(
            ops,
            vec![
                OP_HEADER,
                OP_SCHEMA,
                OP_CHANNEL,
                OP_MESSAGE,
                OP_MESSAGE,
                OP_DATA_END,
                OP_SCHEMA,
                OP_CHANNEL,
                OP_STATISTICS,
                OP_SUMMARY_OFFSET,
                OP_SUMMARY_OFFSET,
                OP_SUMMARY_OFFSET,
                OP_FOOTER,
            ]
        );
    }
}
