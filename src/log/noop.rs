//! No-op macro replacements used when the `defmt` feature is disabled.
//!
//! Each macro accepts any token tree and expands to nothing, producing
//! no code and no flash strings in release builds.

macro_rules! trace {
    ($($t:tt)*) => {};
}
macro_rules! debug {
    ($($t:tt)*) => {};
}
macro_rules! info {
    ($($t:tt)*) => {};
}
macro_rules! noop_warn {
    ($($t:tt)*) => {};
}

pub(crate) use debug;
pub(crate) use info;
pub(crate) use noop_warn as warn;
pub(crate) use trace;
