//! Pure, HAL-free logic shared with the Keychron Q6 HE firmware.
//!
//! Everything here is `no_std` and free of `embassy`/`embedded-hal`
//! dependencies so it can be unit-tested on the host. The firmware crate
//! re-exports these items; this crate is the single source of truth for the
//! transfer-function LUT, the EEPROM byte (de)serialisation primitives, and
//! the small calibration arithmetic helpers.
#![no_std]
#![feature(
    const_convert,
    const_trait_impl,
    const_cmp,
    const_index,
    const_option_ops,
    const_result_trait_fn,
    optimize_attribute
)]
// The test harness needs `std`; the library itself stays `no_std`.
#[cfg(all(test, not(target_os = "none")))]
extern crate std;

pub mod bytes;
pub mod lut;
pub mod math;
