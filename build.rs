//! Build script for the RMK Q6 HE firmware.
//!
//! Sets the required linker arguments for the Cortex-M4 target and gates
//! the firmware build on the q6-core host tests passing.
//!
//! Errors propagate as a tag-only [`BuildError`] returned from `main`,
//! with the underlying detail (env-var error, OS error, failing test
//! output) emitted via `cargo:warning=` lines beforehand. Rust's
//! runtime prints the `Debug` tag to stderr and exits non-zero — no
//! `panic!`, no `expect`, no `Display`/`Error` impls (the latter
//! would force overriding deprecated `Error` defaults or matching
//! patterns that conflict with `clippy::ref_patterns`).

use std::{
    env::{var, var_os},
    process::Command,
};

/// Tag returned from `main` to abort the firmware build. The actionable
/// detail (env var name, underlying `io::Error`, failing test output) is
/// emitted as `cargo:warning=` lines before the tag is returned, so the
/// Debug-printed tag on stderr is just a short identifier.
#[derive(Debug)]
enum BuildError {
    /// A required cargo env var was not set.
    MissingCargoEnv,
    /// The prerequisite test suite ran to completion but reported failures.
    Q6CoreTestsFailed,
    /// Spawning `cargo test` failed at the OS level.
    SpawnCargoTest,
}

/// Entry point for the build script.
///
/// Registers change detection for `memory.x` and the layout feature flags,
/// sets the required linker arguments, then runs the q6-core host test
/// suite. Returns `Err` on any failure; cargo prints it and exits non-zero.
fn main() -> Result<(), BuildError> {
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

    run_q6_core_tests()
}

/// Compile and run the q6-core host test suite. Returns `Err` on any
/// failure; cargo treats that as a build abort.
///
/// Set `SKIP_Q6_CORE_TESTS=1` to bypass (useful during fast iteration when
/// you know the tests already passed in this session).
fn run_q6_core_tests() -> Result<(), BuildError> {
    if var_os("SKIP_Q6_CORE_TESTS").is_some() {
        println!("cargo:warning=skipping q6-core host tests (SKIP_Q6_CORE_TESTS set)");
        return Ok(());
    }

    // Re-run when any q6-core source changes.
    println!("cargo:rerun-if-changed=q6-core/src");
    println!("cargo:rerun-if-changed=q6-core/tests");
    println!("cargo:rerun-if-changed=q6-core/Cargo.toml");

    let manifest_dir = match var("CARGO_MANIFEST_DIR") {
        Ok(value) => value,
        Err(error) => {
            println!("cargo:warning=cargo did not set CARGO_MANIFEST_DIR ({error})");
            return Err(BuildError::MissingCargoEnv);
        },
    };
    let host_triple = match var("HOST") {
        Ok(value) => value,
        Err(error) => {
            println!("cargo:warning=cargo did not set HOST ({error})");
            return Err(BuildError::MissingCargoEnv);
        },
    };
    let cargo = var("CARGO").unwrap_or_else(|_| "cargo".to_owned());

    let q6_core_manifest = format!("{manifest_dir}/q6-core/Cargo.toml");
    // Separate target dir so the test run does not contend with the
    // in-flight firmware build's locks.
    let test_target_dir = format!("{manifest_dir}/target/q6-core-host-tests");

    let spawn_result = Command::new(&cargo)
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
        .output();
    let output = match spawn_result {
        Ok(value) => value,
        Err(error) => {
            println!("cargo:warning=failed to spawn `cargo test` for q6-core ({error})");
            return Err(BuildError::SpawnCargoTest);
        },
    };

    if output.status.success() {
        return Ok(());
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    // Surface the failing test output to the user via cargo warnings;
    // build-script stdout/stderr is otherwise hidden by default.
    for line in stdout.lines().chain(stderr.lines()) {
        println!("cargo:warning={line}");
    }
    Err(BuildError::Q6CoreTestsFailed)
}
