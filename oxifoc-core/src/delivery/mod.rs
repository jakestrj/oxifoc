//! Delivery semantics — a typed ladder of delivery guarantees.
//!
//! Retain the pure retry policy and delivery class markers without any ergot dependency.

mod policy;

pub use policy::{Decision, GiveUp, Outcome, RetryPolicy, decide};

pub use crate::types::{Keyed, ReqId};

mod sealed {
    pub trait Sealed {}
}

/// Where a command sits on the delivery-guarantee ladder.
pub trait DeliveryClass: sealed::Sealed {}

/// Read or absolute setpoint: `f(f(x)) == f(x)`. Safe to retry blindly.
pub enum Idempotent {}
/// Side-effecting action made safe to retry by server-side dedup on a [`ReqId`].
pub enum Deduplicated {}
/// A bare action: retry re-executes it, and it cannot be deduplicated. The
/// ceiling is at-most-once.
pub enum AtMostOnce {}

impl sealed::Sealed for Idempotent {}
impl sealed::Sealed for Deduplicated {}
impl sealed::Sealed for AtMostOnce {}
impl DeliveryClass for Idempotent {}
impl DeliveryClass for Deduplicated {}
impl DeliveryClass for AtMostOnce {}

/// The outcome of a reliable send, told in terms of *what the caller may
/// conclude about whether the effect happened* — this taxonomy is the guarantee.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeliveryError {
    /// The frame never left the host (no route / local queue full). The effect
    /// definitely did **not** happen — safe to re-send even for [`AtMostOnce`].
    NotSent,
    /// Sent, but no response within the budget. The effect **may** have
    /// happened. `attempts` is how many sends were made.
    TimedOut { attempts: u32 },
    /// The server processed the request and replied with a protocol/application
    /// error. Authoritative — never retried.
    Remote,
    /// Aborted by the caller (cancellation / reconnect).
    Cancelled,
}
