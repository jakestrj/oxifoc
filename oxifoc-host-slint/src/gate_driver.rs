//! Infineon EiceDRIVER 1ED38x0Mc12M gate driver telemetry UI integration.

use super::{App, GateDriverSwitchRow, SharedRuntime, faults::apply_faults};
use oxifoc_core::types::{
    FaultCategory, FaultClearTarget, GateDriverSnapshot, GateDriverSwitch,
    GateDriverSwitchTelemetry,
};
use oxifoc_host_lib::ops;
use slint::{ComponentHandle, ModelRc, SharedString, VecModel};
use std::thread;

fn format_switch_row(sw: &GateDriverSwitchTelemetry, populated: bool) -> GateDriverSwitchRow {
    let switch_id = sw.switch_id;
    let addr_str = format!("0x{:02X}", switch_id.i2c_addr());

    if !populated {
        return GateDriverSwitchRow {
            name: SharedString::from(switch_id.label()),
            addr: SharedString::from(addr_str),
            status: SharedString::from("Waiting…"),
            has_fault: false,
            is_ready: false,
            pins: SharedString::from("—"),
            rdy_flags: SharedString::from("—"),
            vcc2: SharedString::from("—"),
            gnd2: SharedString::from("—"),
            vdif: SharedString::from("—"),
            die_temp: SharedString::from("—"),
            fault_events: SharedString::from("—"),
            status_regs: SharedString::from("—"),
            fault_counts: SharedString::from("—"),
        };
    }

    let status = if !sw.configured {
        "UNCONFIG"
    } else if !sw.i2c_ok {
        "I2C ERR"
    } else if sw.fltevt != 0 || sw.has_gflt() || sw.secuvevt != 0 || sw.comerrst != 0 {
        "FAULT"
    } else if !sw.is_ready() {
        "NOT RDY"
    } else {
        "READY"
    };

    let pins = format!(
        "FLT_N={} RDYC={}",
        if sw.flt_n { "1" } else { "0" },
        if sw.rdyc { "1" } else { "0" },
    );

    let rdy_flags = format!(
        "C:{} P:{} S:{}",
        if sw.chip_rdy { "1" } else { "0" },
        if sw.pri_rdy { "1" } else { "0" },
        if sw.sec_rdy { "1" } else { "0" },
    );

    let (vcc2, gnd2, vdif, die_temp) = if sw.i2c_ok {
        (
            format!("{:.2} V", sw.adcmvcc2),
            format!("{:.2} V", sw.adcmgnd2),
            format!("{:.2} V", sw.adcmvdif),
            format!("{:.1} °C", sw.adcmtemp),
        )
    } else {
        (
            "—".to_string(),
            "—".to_string(),
            "—".to_string(),
            "—".to_string(),
        )
    };

    let fault_events = format!(
        "FLT:0x{:02X} GFLT:0x{:02X} UV:0x{:02X}",
        sw.fltevt, sw.gfltevt, sw.secuvevt
    );

    let status_regs = format!(
        "PIN:0x{:02X} CHIP:0x{:02X} COM:0x{:02X} STICK:0x{:02X}",
        sw.pinstat, sw.chipstat, sw.comerrst, sw.evtstick
    );

    let fault_counts = format!("UV1:{} UV2:{} D2E:{}", sw.uv1fcnt, sw.uv2fcnt, sw.d2ecnt);

    GateDriverSwitchRow {
        name: SharedString::from(switch_id.label()),
        addr: SharedString::from(addr_str),
        status: SharedString::from(status),
        has_fault: sw.has_fault(),
        is_ready: sw.is_ready(),
        pins: SharedString::from(pins),
        rdy_flags: SharedString::from(rdy_flags),
        vcc2: SharedString::from(vcc2),
        gnd2: SharedString::from(gnd2),
        vdif: SharedString::from(vdif),
        die_temp: SharedString::from(die_temp),
        fault_events: SharedString::from(fault_events),
        status_regs: SharedString::from(status_regs),
        fault_counts: SharedString::from(fault_counts),
    }
}

/// Build the initial placeholder rows for all 6 switches (`UH`..`WL`).
pub(super) fn default_rows() -> Vec<GateDriverSwitchRow> {
    GateDriverSnapshot::default()
        .switches
        .iter()
        .map(|sw| format_switch_row(sw, false))
        .collect()
}

/// Push an aggregated [`GateDriverSnapshot`] into the Slint UI properties.
pub(super) fn apply_gate_driver_snapshot(app: &App, snap: &GateDriverSnapshot) {
    let rows: Vec<GateDriverSwitchRow> = snap
        .switches
        .iter()
        .enumerate()
        .map(|(idx, sw)| {
            let populated = (snap.populated_mask & (1u8 << idx)) != 0;
            format_switch_row(sw, populated)
        })
        .collect();

    let ready = snap.ready_count();
    let any_fault = snap.any_fault();
    let populated_count = snap.populated_mask.count_ones() as usize;

    let summary = if populated_count == 0 {
        "—".to_string()
    } else if any_fault {
        format!("FAULT ({ready}/{} RDY)", GateDriverSwitch::COUNT)
    } else {
        format!("{ready}/{} READY", GateDriverSwitch::COUNT)
    };

    app.set_gd_switches(ModelRc::new(VecModel::from(rows)));
    app.set_gd_summary_text(SharedString::from(summary));
    app.set_gd_has_fault(any_fault);
    app.set_gd_seq_text(SharedString::from(format!("{}", snap.seq)));

    if snap.board_temp_c_x10 != 0 && snap.board_temp_c_x10 > -3000 {
        app.set_board_temp_text(SharedString::from(format!(
            "{:.1} °C",
            f32::from(snap.board_temp_c_x10) / 10.0
        )));
    }
    if let Some(max_die) = snap.max_die_temp_c() {
        app.set_gd_max_die_temp_text(SharedString::from(format!("{max_die:.1} °C")));
    }
}

pub(super) fn bind(app: &App, runtime: &SharedRuntime) {
    let rt = runtime.clone();
    let weak = app.as_weak();
    app.on_gd_clear_faults(move || {
        let cmd_tx = {
            let guard = rt.lock().unwrap();
            match guard.as_ref() {
                Some(r) => r.cmd_tx.clone(),
                None => return,
            }
        };
        let weak = weak.clone();
        thread::spawn(move || {
            match ops::fault::clear(
                &cmd_tx,
                FaultClearTarget::Category(FaultCategory::DriverFault),
            ) {
                Ok(snapshot) => {
                    let _ = weak.upgrade_in_event_loop(move |app| apply_faults(&app, &snapshot));
                }
                Err(error) => {
                    tracing::warn!("Gate driver fault clear failed: {error:#}");
                    if let Ok(snapshot) = ops::fault::query(&cmd_tx) {
                        let _ =
                            weak.upgrade_in_event_loop(move |app| apply_faults(&app, &snapshot));
                    }
                }
            }
        });
    });
}
