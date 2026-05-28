//! Build script for the RMK Q6 HE firmware.
//!
//! Sets the required linker arguments for the Cortex-M4 target and gates
//! the firmware build on the q6-core host tests passing.
//!
//! Errors propagate as a typed [`BuildError`] returned from `main`, so
//! cargo prints them to stderr and exits non-zero on its own — no
//! `panic!` or `expect` involved.
#![expect(
    clippy::question_mark_used,
    clippy::missing_trait_methods,
    clippy::pattern_type_mismatch,
    reason = "`?` is the idiomatic alternative to expect/panic the project requires; the deprecated `Error::{description, cause}` and internal `provide`/`type_id` defaults must NOT be overridden; `pattern_type_mismatch` contradicts `ref_patterns` for non-Copy enum variants (cannot satisfy both)"
)]

use core::error::Error;
use core::fmt;
use std::env::{VarError, var, var_os};
use std::io;
use std::process::Command;

/// Errors the build script surfaces back to cargo.
///
/// Variants are listed alphabetically (per `clippy::arbitrary_source_item_ordering`).
#[derive(Debug)]
enum BuildError {
    /// Cargo did not set an env var it is contractually required to set.
    MissingCargoEnv {
        /// Name of the missing env var, e.g. `"CARGO_MANIFEST_DIR"`.
        name:   &'static str,
        /// Underlying [`VarError`] (`NotPresent` or `NotUnicode`).
        source: VarError,
    },
    /// The prerequisite test suite ran to completion but reported failures.
    Q6CoreTestsFailed,
    /// Spawning `cargo test` failed at the OS level (binary not on `PATH`,
    /// fork/exec failed, etc.).
    SpawnCargoTest(io::Error),
}

impl fmt::Display for BuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingCargoEnv { name, .. } => write!(f, "cargo did not set the `{name}` environment variable"),
            Self::Q6CoreTestsFailed => {
                write!(f, "q6-core host tests failed; firmware build aborted. Set SKIP_Q6_CORE_TESTS=1 to bypass.")
            },
            Self::SpawnCargoTest(err) => write!(f, "failed to spawn `cargo test` for q6-core: {err}"),
        }
    }
}

impl Error for BuildError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::MissingCargoEnv { source, .. } => Some(source),
            Self::Q6CoreTestsFailed => None,
            Self::SpawnCargoTest(err) => Some(err),
        }
    }
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

    let manifest_dir = var("CARGO_MANIFEST_DIR")
        .map_err(|source| BuildError::MissingCargoEnv { name: "CARGO_MANIFEST_DIR", source })?;
    let host_triple = var("HOST").map_err(|source| BuildError::MissingCargoEnv { name: "HOST", source })?;
    let cargo = var("CARGO").unwrap_or_else(|_| "cargo".to_owned());

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
        .map_err(BuildError::SpawnCargoTest)?;

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
