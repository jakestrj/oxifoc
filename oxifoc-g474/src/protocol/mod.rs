//! Protocol layer for `streaming-protocol` + `nmea` communication over CAN.

pub mod servers;

use core::sync::atomic::{AtomicU8, Ordering};

/// High-level device states for LED status indication
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    /// Device is booting / initializing
    Boot = 0,
    /// Waiting for CAN link / first command
    WaitingLink = 1,
    /// Active communication link established
    Linked = 2,
    /// Error state (fault active)
    Error = 3,
}

static DEVICE_STATE: AtomicU8 = AtomicU8::new(DeviceState::Boot as u8);

/// Set current device state
pub fn set_device_state(state: DeviceState) {
    DEVICE_STATE.store(state as u8, Ordering::Relaxed);
}

/// Get current device state
pub fn get_device_state() -> DeviceState {
    match DEVICE_STATE.load(Ordering::Relaxed) {
        0 => DeviceState::Boot,
        1 => DeviceState::WaitingLink,
        2 => DeviceState::Linked,
        _ => DeviceState::Error,
    }
}
