//! End-to-end integration tests for `oxifoc-virtual` over Virtual CAN (`vcan` multicast)
//! using `streaming-protocol` and `nmea` (`NMEA_RX_ARBITRATION_ID = 0x7E5`,
//! `NMEA_TX_ARBITRATION_ID = 0x7ED`).

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use femtopb::{EnumValue, UnknownFields};
use oxifoc_core::foc::phase::PhaseSource;
use oxifoc_core::icd::{
    FaultStateSnapshot, MotorRequest, NMEA_RX_ARBITRATION_ID, NMEA_TX_ARBITRATION_ID,
    VCAN_MULTICAST_IP, vcan_multicast_port,
};
use oxifoc_core::proto::oxifoc::{
    FaultCategory as ProtoFaultCategory, FaultClearRequest, TelemetryConfigRequest, WireMessage,
    wire_message::Payload,
};
use oxifoc_core::runtime::servers::{CommsError, NmeaCanReceiver, encode_wire_message_frames};
use oxifoc_core::types::{
    ControlMode, FastTelemetry, GateDriverSwitchTelemetry, MotorCommand, MotorState, SlowTelemetry,
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

#[derive(Debug)]
enum DecodedTelemetry {
    Slow(SlowTelemetry),
    Fast(FastTelemetry),
    Fault(#[allow(dead_code)] FaultStateSnapshot),
    GateDriver(#[allow(dead_code)] GateDriverSwitchTelemetry),
}

fn decode_rx_frame(rx: &mut NmeaCanReceiver, frame_bytes: &[u8]) -> Option<DecodedTelemetry> {
    rx.push_frame_with(frame_bytes, |proto| match proto.payload.as_ref() {
        Some(Payload::SlowTelemetry(s)) => Ok(DecodedTelemetry::Slow(SlowTelemetry::from(s))),
        Some(Payload::FastTelemetry(f)) => Ok(DecodedTelemetry::Fast(FastTelemetry::from(f))),
        Some(Payload::FaultState(flt)) => {
            Ok(DecodedTelemetry::Fault(FaultStateSnapshot::from(flt)))
        }
        Some(Payload::GateDriverTelemetry(gd)) => Ok(DecodedTelemetry::GateDriver(
            GateDriverSwitchTelemetry::from(gd),
        )),
        _ => Err(CommsError::InvalidRequest),
    })
    .ok()
    .flatten()
}

#[test]
fn e2e_can_streaming_protocol_over_vcan_multicast() {
    let can_iface = format!("mcast:vcan_e2e_{}", std::process::id());
    let mcast_port = vcan_multicast_port(&can_iface);
    let mcast_ip = Ipv4Addr::from(VCAN_MULTICAST_IP);
    let mcast_addr = SocketAddrV4::new(mcast_ip, mcast_port);

    let child = Command::new(env!("CARGO_BIN_EXE_oxifoc-virtual"))
        .args([
            "--transport",
            "vcan",
            "--can-interface",
            &can_iface,
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
        .expect("spawn oxifoc-virtual on vcan multicast");
    let _guard = ChildGuard(child);

    // Give the virtual CAN server a moment to bind and join multicast
    std::thread::sleep(Duration::from_millis(150));

    let sock = socket2::Socket::new(
        socket2::Domain::IPV4,
        socket2::Type::DGRAM,
        Some(socket2::Protocol::UDP),
    )
    .expect("create multicast test socket");
    sock.set_reuse_address(true).unwrap();
    #[cfg(unix)]
    sock.set_reuse_port(true).unwrap();
    sock.set_multicast_loop_v4(true).unwrap();
    sock.set_multicast_if_v4(&Ipv4Addr::LOCALHOST).unwrap();
    sock.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, mcast_port).into())
        .expect("bind multicast test socket");
    sock.join_multicast_v4(&mcast_ip, &Ipv4Addr::LOCALHOST)
        .expect("join multicast group");
    let socket: UdpSocket = sock.into();
    socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();

    let send_mcast_msg = |msg: &WireMessage<'_>| {
        let mut nmea_msg = encode_wire_message_frames(msg).expect("encode wire message");
        let id_bytes = NMEA_RX_ARBITRATION_ID.to_le_bytes();
        while let Some(frame) = nmea_msg.pop_frame() {
            let mut wire = [0u8; CAN_WIRE_FRAME_LEN];
            wire[0..2].copy_from_slice(&id_bytes);
            wire[2..10].copy_from_slice(&frame.bytes);
            socket
                .send_to(&wire, mcast_addr)
                .expect("send CAN datagram to multicast bus");
        }
    };

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
        send_mcast_msg(&telem_cfg);
        let motor_req = MotorRequest {
            source_session: 0x1234,
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
        send_mcast_msg(&cmd_msg);

        for _ in 0..25 {
            let mut buf = [0u8; 64];
            if let Ok((len, _)) = socket.recv_from(&mut buf)
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

    assert!(
        got_slow,
        "should receive SlowTelemetry over UDP Multicast Virtual CAN"
    );
    assert!(
        spun,
        "motor should spin in response to CurrentControl command over UDP Multicast Virtual CAN"
    );

    // Send EmergencyStop, PhaseSourceRequest, and FaultClearRequest
    let estop_req = MotorRequest {
        source_session: 0x1234,
        seq,
        command: MotorCommand::EmergencyStop,
    };
    let estop_msg = WireMessage {
        payload: Some(Payload::MotorCommand(estop_req.into())),
        unknown_fields: UnknownFields::empty(),
    };
    send_mcast_msg(&estop_msg);

    let ps_msg = WireMessage {
        payload: Some(Payload::PhaseSourceRequest(
            PhaseSource::Hall.to_proto_request(),
        )),
        unknown_fields: UnknownFields::empty(),
    };
    send_mcast_msg(&ps_msg);

    let fault_clear_msg = WireMessage {
        payload: Some(Payload::FaultClear(FaultClearRequest {
            expected_generation: 0,
            clear_all: true,
            category: EnumValue::Known(ProtoFaultCategory::None),
            unknown_fields: UnknownFields::empty(),
        })),
        unknown_fields: UnknownFields::empty(),
    };
    send_mcast_msg(&fault_clear_msg);

    let mut stopped = false;
    let stop_deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < stop_deadline && !stopped {
        let mut buf = [0u8; 64];
        if let Ok((len, _)) = socket.recv_from(&mut buf)
            && len == CAN_WIRE_FRAME_LEN
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
fn e2e_host_lib_to_virtual_over_vcan_multicast() {
    use oxifoc_core::icd::TelemetryConfig;
    use oxifoc_host_lib::{HostCommand, HostConfig, ReconnectPolicy, TransportType, start_host};

    let can_iface = format!("mcast:vcan_host_e2e_{}", std::process::id());

    let child = Command::new(env!("CARGO_BIN_EXE_oxifoc-virtual"))
        .args([
            "--transport",
            "vcan",
            "--can-interface",
            &can_iface,
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
        .expect("spawn oxifoc-virtual on vcan multicast for host-lib test");
    let _guard = ChildGuard(child);

    let cfg = HostConfig {
        transport: Some(TransportType::Can),
        can_interface: Some(can_iface),
        reconnect: Some(ReconnectPolicy::Limited(5)),
        fast_hz: Some(200),
    };

    let host = start_host(cfg);
    assert!(
        host.wait_for_connection(Duration::from_secs(5)),
        "oxifoc-host-lib should connect to oxifoc-virtual over UDP Multicast Virtual CAN"
    );

    host.cmd_tx
        .send(HostCommand::SetTelemetryConfig(
            TelemetryConfig { fast_hz: 200 },
            None,
        ))
        .expect("send SetTelemetryConfig");
    host.cmd_tx
        .send(HostCommand::Motor(ControlMode::CurrentControl {
            iq_target: 4.0,
            id_target: 0.0,
        }))
        .expect("send Motor CurrentControl");

    let mut got_slow = false;
    let mut got_gd = false;
    let mut spun = false;
    let deadline = Instant::now() + Duration::from_secs(8);

    while Instant::now() < deadline && !(got_slow && got_gd && spun) {
        for slow in host.slow_rx.try_iter() {
            if slow.vbus_mv == 24_000 {
                got_slow = true;
            }
        }
        for gd in host.gd_rx.try_iter() {
            if gd.populated_mask != 0 && gd.ready_count() > 0 {
                got_gd = true;
            }
        }
        for fast in host.fast_rx.try_iter() {
            if fast.rpm.abs() > 10 {
                spun = true;
            }
        }
        std::thread::sleep(Duration::from_millis(25));
    }

    assert!(
        got_slow,
        "oxifoc-host-lib should receive SlowTelemetry from oxifoc-virtual over vcan"
    );
    assert!(
        got_gd,
        "oxifoc-host-lib should receive GateDriverSnapshot from oxifoc-virtual over vcan"
    );
    assert!(
        spun,
        "oxifoc-host-lib should command motor spin on oxifoc-virtual over vcan"
    );

    host.shutdown();
}
