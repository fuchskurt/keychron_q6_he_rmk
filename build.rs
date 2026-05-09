//! Build script for the RMK Q6 HE firmware.
//!
//! Sets the required linker arguments for the Cortex-M4 target.

/// Entry point for the build script.
///
/// Registers change detection for `memory.x` and sets the required linker
/// arguments.
fn main() {
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ANSI_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ISO_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_JIS_LAYOUT");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
}
