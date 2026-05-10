//! Logging abstraction gated behind the `defmt` feature flag.
//!
//! Provides uniform logging macros (`trace!`, `debug!`, `info!`, `warn!`)
//! that are available across the entire crate without scattering
//! `#[cfg(feature = "defmt")]` at every call site.
//!
//! # With the `defmt` feature enabled
//!
//! All macros delegate directly to [`defmt`] and output over RTT via
//! [`defmt_rtt`]. Messages are visible in `probe-rs` or any RTT host.
//!
//! # Without the `defmt` feature
//!
//! All macros expand to nothing. No strings are stored in flash, no code
//! is generated, and there is zero runtime overhead.
//!
//! # Usage
//!
//! ```rust
//! use crate::log::debug;
//! debug!("raw={}", raw);
//! ```

#[cfg(feature = "defmt")]
pub(crate) use defmt::{debug, info, trace, warn};

#[cfg(not(feature = "defmt"))] mod noop;

#[cfg(not(feature = "defmt"))]
pub(crate) use noop::*;
