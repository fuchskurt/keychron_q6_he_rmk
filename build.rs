//! Build script for the RMK Q6 HE firmware.
//!
//! Sets the required linker arguments for the Cortex-M4 target and gates
//! the firmware build on the q6-core host tests passing.

use std::env::{var, var_os};
use std::process::Command;

/// Entry point for the build script.
///
/// Registers change detection for `memory.x` and the layout feature flags,
/// sets the required linker arguments, then runs the q6-core host test
/// suite. If any test fails the firmware build is aborted.
fn main() {
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ANSI_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ISO_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_JIS_LAYOUT");
    println!("cargo:rerun-if-env-changed=SKIP_Q6_CORE_TESTS");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    if var_os("CARGO_FEATURE_DEFMT").is_some() {
        println!("cargo:rustc-link-arg=-Tdefmt.x");
    }

    run_q6_core_tests();
}

/// Compile and run the q6-core host test suite. Aborts the firmware build
/// on any failure.
///
/// Set `SKIP_Q6_CORE_TESTS=1` to bypass (useful during fast iteration when
/// you know the tests already passed in this session).
fn run_q6_core_tests() {
    if var_os("SKIP_Q6_CORE_TESTS").is_some() {
        println!("cargo:warning=skipping q6-core host tests (SKIP_Q6_CORE_TESTS set)");
        return;
    }

    // Re-run when any q6-core source changes.
    println!("cargo:rerun-if-changed=q6-core/src");
    println!("cargo:rerun-if-changed=q6-core/tests");
    println!("cargo:rerun-if-changed=q6-core/Cargo.toml");

    let manifest_dir = var("CARGO_MANIFEST_DIR").expect("cargo did not set CARGO_MANIFEST_DIR");
    let host_triple = var("HOST").expect("cargo did not set HOST");
    let cargo = var("CARGO").unwrap_or_else(|_| "cargo".to_string());

    let q6_core_manifest = format!("{manifest_dir}/q6-core/Cargo.toml");
    // Separate target dir so the test run does not contend with the
    // in-flight firmware build's locks.
    let test_target_dir = format!("{manifest_dir}/target/q6-core-host-tests");

    let output = Command::new(&cargo)
        .args([
            "test",
            "-Zbuild-std=std",
            "--manifest-path",
            &q6_core_manifest,
            "--target",
            &host_triple,
            "--target-dir",
            &test_target_dir,
            "--quiet",
        ])
        // Workspace .cargo/config.toml sets `build-std` for the embedded
        // target; for the host test run we override it to an empty list
        // so the toolchain's prebuilt `std` is used. Explicit `--config`
        // takes precedence over both the workspace `config.toml` and any
        // inherited env var.
        .args(["--config", "unstable.build-std=[]"])
        // The firmware's RUSTFLAGS (if any) are not appropriate for the
        // host-target test build.
        .env_remove("RUSTFLAGS")
        .env_remove("CARGO_BUILD_TARGET")
        .output()
        .expect("failed to spawn `cargo test` for q6-core");

    if !output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        // Surface the failing test output to the user via cargo warnings;
        // build-script stdout/stderr is otherwise hidden by default.
        for line in stdout.lines().chain(stderr.lines()) {
            println!("cargo:warning={line}");
        }
        panic!("q6-core host tests failed; firmware build aborted. Set SKIP_Q6_CORE_TESTS=1 to bypass.");
    }
}
