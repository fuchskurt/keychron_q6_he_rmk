//! Build script for the RMK Q6 HE firmware.
//!
//! Registers the layout feature flags for change detection and
//! sets the linker arguments required by the Cortex-M4 target.

fn main() {
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ANSI_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ISO_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_JIS_LAYOUT");
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
}
