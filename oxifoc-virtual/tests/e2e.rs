//! End-to-end integration tests for `oxifoc-virtual` over CAN-framed TCP and UDP
//! using `streaming-protocol` and `nmea` (`NMEA_RX_ARBITRATION_ID = 0x7E5`,
//! `NMEA_TX_ARBITRATION_ID = 0x7ED`).

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, UdpSocket};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use femtopb::{EnumValue, UnknownFields};
use oxifoc_core::foc::phase::PhaseSource;
use oxifoc_core::icd::{
    FaultStateSnapshot, MotorRequest, NMEA_RX_ARBITRATION_ID, NMEA_TX_ARBITRATION_ID,
};
use oxifoc_core::proto::oxifoc::{
    FaultCategory as ProtoFaultCategory, FaultClearRequest, TelemetryConfigRequest, WireMessage,
    wire_message::Payload,
};
use oxifoc_core::runtime::servers::{
    CommsError, NmeaCanReceiver, encode_wire_message_frames,
};
use oxifoc_core::types::{
    ControlMode, FastTelemetry, MotorCommand, MotorState, SlowTelemetry,
};

const CAN_WIRE_FRAME_LEN: usize = 10;

/// Kills the spawned virtual device when the test ends (even on panic).
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn free_tcp_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn free_udp_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn spawn_device(transport_arg: &str, port: u16) -> ChildGuard {
    let child = Command::new(env!("CARGO_BIN_EXE_oxifoc-virtual"))
        .args([
            "--transport",
            transport_arg,
            "--port",
            &port.to_string(),
            "--vbus",
            "24",
            "--pole-pairs",
            "7",
            "--max-current",
            "20",
            "--load",
            "0.2",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn oxifoc-virtual");
    ChildGuard(child)
}

#[derive(Debug)]
enum DecodedTelemetry {
    Slow(SlowTelemetry),
    Fast(FastTelemetry),
    Fault(#[allow(dead_code)] FaultStateSnapshot),
}

fn decode_rx_frame(rx: &mut NmeaCanReceiver, frame_bytes: &[u8]) -> Option<DecodedTelemetry> {
    rx.push_frame_with(frame_bytes, |proto| match proto.payload.as_ref() {
        Some(Payload::SlowTelemetry(s)) => Ok(DecodedTelemetry::Slow(SlowTelemetry::from(s))),
        Some(Payload::FastTelemetry(f)) => Ok(DecodedTelemetry::Fast(FastTelemetry::from(f))),
        Some(Payload::FaultState(flt)) => Ok(DecodedTelemetry::Fault(FaultStateSnapshot::from(flt))),
        _ => Err(CommsError::InvalidRequest),
    })
    .ok()
    .flatten()
}

fn send_tcp_wire_message(stream: &mut TcpStream, msg: &WireMessage<'_>) {
    let mut nmea_msg = encode_wire_message_frames(msg).expect("encode wire message");
    let id_bytes = NMEA_RX_ARBITRATION_ID.to_le_bytes();
    while let Some(frame) = nmea_msg.pop_frame() {
        let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
        wire[0..2].copy_from_slice(&id_bytes);
        wire[2..10].copy_from_slice(&frame.bytes);
        stream.write_all(&wire).expect("write CAN frame over TCP");
    }
}

fn send_udp_wire_message(socket: &UdpSocket, msg: &WireMessage<'_>) {
    let mut nmea_msg = encode_wire_message_frames(msg).expect("encode wire message");
    let id_bytes = NMEA_RX_ARBITRATION_ID.to_le_bytes();
    while let Some(frame) = nmea_msg.pop_frame() {
        let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
        wire[0..2].copy_from_slice(&id_bytes);
        wire[2..10].copy_from_slice(&frame.bytes);
        socket.send(&wire).expect("send CAN datagram over UDP");
    }
}

#[test]
fn e2e_can_streaming_protocol_over_tcp() {
    let port = free_tcp_port();
    let _guard = spawn_device("tcp", port);

    let deadline = Instant::now() + Duration::from_secs(10);
    let mut stream = loop {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(s) => break s,
            Err(_) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(25));
            }
            Err(e) => panic!("virtual TCP listener did not start: {e}"),
        }
    };
    stream
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();

    // 1. Enable fast telemetry at 200 Hz
    let telem_cfg = WireMessage {
        payload: Some(Payload::TelemetryConfig(TelemetryConfigRequest {
            fast_hz: 200,
            unknown_fields: UnknownFields::empty(),
        })),
        unknown_fields: UnknownFields::empty(),
    };
    send_tcp_wire_message(&mut stream, &telem_cfg);

    // 2. Wait for SlowTelemetry showing 24V bus voltage once the FOC loop has ticked
    let mut nmea_rx = NmeaCanReceiver::new();
    let mut got_slow = false;
    let wait_deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < wait_deadline && !got_slow {
        let mut buf = [0u8; CAN_WIRE_FRAME_LEN];
        if stream.read_exact(&mut buf).is_ok() {
            let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
            assert_eq!(arb_id, NMEA_TX_ARBITRATION_ID);
            if let Some(DecodedTelemetry::Slow(slow)) = decode_rx_frame(&mut nmea_rx, &buf[2..10])
                && slow.vbus_mv == 24_000
            {
                got_slow = true;
            }
        }
    }
    assert!(got_slow, "should receive SlowTelemetry with 24V vbus over TCP");

    // 3. Command motor current control (4.0 A Iq) and verify motor spins up
    let mut seq = 1u32;
    let mut spun = false;
    let spin_deadline = Instant::now() + Duration::from_secs(8);
    while Instant::now() < spin_deadline && !spun {
        let motor_req = MotorRequest {
            source_session: 0xCAFE,
            seq,
            command: MotorCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 4.0,
                id_target: 0.0,
            }),
        };
        seq = seq.wrapping_add(1);
        let cmd_msg = WireMessage {
            payload: Some(Payload::MotorCommand(motor_req.into())),
            unknown_fields: UnknownFields::empty(),
        };
        send_tcp_wire_message(&mut stream, &cmd_msg);

        for _ in 0..20 {
            let mut buf = [0u8; CAN_WIRE_FRAME_LEN];
            if stream.read_exact(&mut buf).is_ok() {
                let arb_id = u16::from_le_bytes([buf[0], buf[1]]);
                if arb_id == NMEA_TX_ARBITRATION_ID
                    && let Some(decoded) = decode_rx_frame(&mut nmea_rx, &buf[2..10])
                {
                    match decoded {
                        DecodedTelemetry::Fast(fast) if fast.rpm.abs() > 10 => {
                            spun = true;
                            break;
                        }
                        DecodedTelemetry::Slow(slow)
                            if slow.motor_state == MotorState::Running =>
                        {
                            // Keep reading until FastTelemetry shows non-zero RPM
                        }
                        _ => {}
                    }
                }
            } else {
                break;
            }
        }
    }
    assert!(spun, "motor should spin in response to CurrentControl command over TCP");

    // 4. Send EmergencyStop and PhaseSourceRequest + FaultClearRequest
    let estop_req = MotorRequest {
        source_session: 0xCAFE,
        seq,
        command: MotorCommand::EmergencyStop,
    };
    let estop_msg = WireMessage {
        payload: Some(Payload::MotorCommand(estop_req.into())),
        unknown_fields: UnknownFields::empty(),
    };
    send_tcp_wire_message(&mut stream, &estop_msg);

    let ps_msg = WireMessage {
        payload: Some(Payload::PhaseSourceRequest(
            PhaseSource::Hall.to_proto_request(),
        )),
        unknown_fields: UnknownFields::empty(),
    };
    send_tcp_wire_message(&mut stream, &ps_msg);

    let fault_clear_msg = WireMessage {
        payload: Some(Payload::FaultClear(FaultClearRequest {
            expected_generation: 0,
            clear_all: true,
            category: EnumValue::Known(ProtoFaultCategory::None),
            unknown_fields: UnknownFields::empty(),
        })),
        unknown_fields: UnknownFields::empty(),
    };
    send_tcp_wire_message(&mut stream, &fault_clear_msg);

    let mut stopped = false;
    let stop_deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < stop_deadline && !stopped {
        let mut buf = [0u8; CAN_WIRE_FRAME_LEN];
        if stream.read_exact(&mut buf).is_ok()
            && u16::from_le_bytes([buf[0], buf[1]]) == NMEA_TX_ARBITRATION_ID
            && let Some(DecodedTelemetry::Slow(slow)) = decode_rx_frame(&mut nmea_rx, &buf[2..10])
            && slow.motor_state == MotorState::Stopped
            && slow.control_mode == ControlMode::Stopped
        {
            stopped = true;
        }
    }
    assert!(stopped, "EmergencyStop should transition motor to Stopped");
}

#[test]
fn e2e_can_streaming_protocol_over_udp() {
    let port = free_udp_port();
    let _guard = spawn_device("udp", port);

    // Give the UDP server a moment to bind
    std::thread::sleep(Duration::from_millis(150));

    let socket = UdpSocket::bind("127.0.0.1:0").expect("bind client UDP socket");
    socket
        .connect(("127.0.0.1", port))
        .expect("connect UDP socket");
    socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();

    // Send TelemetryConfigRequest to register our peer address and enable FastTelemetry
    let telem_cfg = WireMessage {
        payload: Some(Payload::TelemetryConfig(TelemetryConfigRequest {
            fast_hz: 200,
            unknown_fields: UnknownFields::empty(),
        })),
        unknown_fields: UnknownFields::empty(),
    };

    let mut nmea_rx = NmeaCanReceiver::new();
    let mut got_slow = false;
    let mut seq = 1u32;
    let mut spun = false;
    let deadline = Instant::now() + Duration::from_secs(8);

    while Instant::now() < deadline && !(got_slow && spun) {
        send_udp_wire_message(&socket, &telem_cfg);
        let motor_req = MotorRequest {
            source_session: 0xBEEF,
            seq,
            command: MotorCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 4.0,
                id_target: 0.0,
            }),
        };
        seq = seq.wrapping_add(1);
        let cmd_msg = WireMessage {
            payload: Some(Payload::MotorCommand(motor_req.into())),
            unknown_fields: UnknownFields::empty(),
        };
        send_udp_wire_message(&socket, &cmd_msg);

        for _ in 0..25 {
            let mut buf = [0u8; 64];
            if let Ok(len) = socket.recv(&mut buf)
                && len == CAN_WIRE_FRAME_LEN
                && u16::from_le_bytes([buf[0], buf[1]]) == NMEA_TX_ARBITRATION_ID
                && let Some(decoded) = decode_rx_frame(&mut nmea_rx, &buf[2..10])
            {
                match decoded {
                    DecodedTelemetry::Slow(slow) if slow.vbus_mv == 24_000 => {
                        got_slow = true;
                    }
                    DecodedTelemetry::Fast(fast) if fast.rpm.abs() > 10 => {
                        spun = true;
                    }
                    _ => {}
                }
            }
        }
    }

    assert!(got_slow, "should receive SlowTelemetry over UDP");
    assert!(spun, "motor should spin in response to CurrentControl command over UDP");
}

#[test]
fn tcp_connection_churn_does_not_kill_server() {
    let port = free_tcp_port();
    let mut guard = spawn_device("tcp", port);

    let deadline = Instant::now() + Duration::from_secs(10);
    let first = loop {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(stream) => break stream,
            Err(_) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(25));
            }
            Err(e) => panic!("virtual TCP listener did not start: {e}"),
        }
    };

    let mut clients = vec![first];
    for _ in 0..8 {
        clients.push(TcpStream::connect(("127.0.0.1", port)).expect("connect churn client"));
        std::thread::sleep(Duration::from_millis(25));
    }
    std::thread::sleep(Duration::from_millis(250));

    assert!(
        guard.0.try_wait().unwrap().is_none(),
        "connection churn must not terminate the virtual device"
    );
}
