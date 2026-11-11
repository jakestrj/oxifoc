//! Sensor implementations for motor control

pub mod as5147;
pub mod current;
pub mod hall;

#[allow(unused_imports)]
pub use as5147::{As5147, As5147AngleProxy, As5147Diagnostics, As5147State};
#[cfg(feature = "board-jbldc")]
pub use as5147::{as5147_task, init_as5147};
pub use current::{F405CurrentSensor, F405CurrentSensorExt};
#[allow(unused_imports)] // Public API not yet wired to protocol handlers
pub use hall::{HallAngleProxy, get_snapshot as get_hall_snapshot, init_hall, read_hall_state_raw};

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;
