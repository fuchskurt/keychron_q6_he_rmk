//! Build script for the RMK Q6 HE ANSI firmware.
//!
//! Generates Vial keyboard configuration constants from `vial.json` and
//! sets the required linker arguments for the Cortex-M4 target.
use core::error::Error;
use fs::read_to_string;
use lzma_rust2::{XzOptions, XzWriter};
use serde_json::{Value, from_str, to_string};
use std::{env, fs, io::Write as _, path::Path};

/// Represents the active keyboard layout selected via Cargo features.
///
/// Exactly one layout must be enabled at compile time:
/// - `ansi_layout`
/// - `iso_layout`
/// - `jis_layout`
///
/// This is used as a single source of truth throughout the build script
/// to select layout-specific resources such as `vial.json` and
/// `VIAL_KEYBOARD_ID`.
#[derive(Copy, Clone)]
enum Layout {
    /// ANSI keyboard layout.
    Ansi,
    /// ISO keyboard layout.
    Iso,
    /// JIS keyboard layout.
    Jis,
}

/// Entry point for the build script.
///
/// Registers change detection for `vial.json` and `memory.x`, sets the
/// required linker arguments, and triggers Vial config generation.
fn main() -> Result<(), Box<dyn Error>> {
    let layout = match detect_layout() {
        Ok(value) => value,
        Err(error) => return Err(error),
    };

    let vial_path = match layout {
        Layout::Ansi => "src/layout/ansi/vial.json",
        Layout::Iso => "src/layout/iso/vial.json",
        Layout::Jis => "src/layout/jis/vial.json",
    };

    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ANSI_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_ISO_LAYOUT");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_JIS_LAYOUT");
    println!("cargo:rerun-if-changed={vial_path}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    generate_vial_config(vial_path, layout)
}

/// Detects the active keyboard layout from Cargo feature environment variables.
///
/// Cargo exposes enabled features as environment variables of the form
/// `CARGO_FEATURE_<FEATURE_NAME>`. This function checks for the presence of:
/// - `CARGO_FEATURE_ANSI_LAYOUT`
/// - `CARGO_FEATURE_ISO_LAYOUT`
/// - `CARGO_FEATURE_JIS_LAYOUT`
///
/// # Errors
///
/// Returns an error if:
/// - none of the layout features are enabled, or
/// - more than one layout feature is enabled.
///
/// In both cases, exactly one layout must be selected to ensure a valid
/// firmware configuration.
fn detect_layout() -> Result<Layout, Box<dyn Error>> {
    let is_iso = env::var("CARGO_FEATURE_ISO_LAYOUT").is_ok();
    let is_ansi = env::var("CARGO_FEATURE_ANSI_LAYOUT").is_ok();
    let is_jis = env::var("CARGO_FEATURE_JIS_LAYOUT").is_ok();

    match (is_ansi, is_iso, is_jis) {
        (true, false, false) => Ok(Layout::Ansi),
        (false, true, false) => Ok(Layout::Iso),
        (false, false, true) => Ok(Layout::Jis),
        _ => Err("Exactly one of `ansi_layout`, `iso_layout`, or `jis_layout` must be enabled".into()),
    }
}

/// Generates the Vial keyboard configuration constants and writes them to
/// `config_generated.rs` in the Cargo output directory.
///
/// The generated file contains:
/// - `VIAL_KEYBOARD_DEF`: XZ-compressed keyboard definition derived from
///   `vial.json`.
/// - `VIAL_KEYBOARD_ID`: Unique 8-byte keyboard identifier.
/// - `VIAL_SERIAL`: Vial serial string derived from the first 4 bytes of the
///   keyboard ID.
fn generate_vial_config(vial_path: &str, layout: Layout) -> Result<(), Box<dyn Error>> {
    let out_dir = match env::var_os("OUT_DIR") {
        Some(value) => value,
        None => return Err("OUT_DIR not set".into()),
    };
    let out_file = Path::new(&out_dir).join("config_generated.rs");

    let vial_cfg = match read_vial_json(vial_path) {
        Ok(value) => value,
        Err(error) => return Err(error),
    };
    let compressed = match compress_vial_cfg(&vial_cfg) {
        Ok(value) => value,
        Err(error) => return Err(error),
    };

    let keyboard_id: [u8; 8] = match layout {
        Layout::Iso => [0xDE, 0x22, 0x4B, 0xDF, 0x09, 0x8D, 0x30, 0x00],
        Layout::Ansi => [0x36, 0x3B, 0x06, 0xF5, 0x13, 0x9F, 0xE4, 0x46],
        Layout::Jis => [0x90, 0x52, 0x87, 0x62, 0x50, 0x77, 0x2C, 0x47],
    };

    match fs::write(out_file, format_constants(&compressed, keyboard_id, &vial_cfg)) {
        Ok(()) => {},
        Err(error) => return Err(error.into()),
    }
    Ok(())
}

/// Reads and minifies `vial.json` from the crate root.
///
/// Parses the JSON and re-stringifies it to strip whitespace before
/// compression. Prints a diagnostic message to stderr if the file cannot
/// be opened, and returns an empty string so compression can still proceed.
fn read_vial_json(path: &str) -> Result<String, Box<dyn Error>> {
    let content = match read_to_string(path) {
        Ok(value) => value,
        Err(error) => {
            return Err(error.into());
        },
    };

    let value: Value = match from_str(&content) {
        Ok(parsed) => parsed,
        Err(error) => {
            return Err(error.into());
        },
    };

    match to_string(&value) {
        Ok(serialized) => Ok(serialized),
        Err(error) => Err(error.into()),
    }
}

/// Compresses a Vial config string using XZ at compression level 9.
///
/// Returns the compressed bytes as a `Vec<u8>` ready to be embedded as a
/// constant in the generated source file.
fn compress_vial_cfg(vial_cfg: &str) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut compressed = Vec::new();

    let result: Result<(), Box<dyn Error>> = {
        let mut writer = match XzWriter::new(&mut compressed, XzOptions::with_preset(9)) {
            Ok(value) => value,
            Err(error) => {
                return Err(error.into());
            },
        }
        .auto_finish();

        match writer.write_all(vial_cfg.as_bytes()) {
            Ok(()) => Ok(()),
            Err(error) => Err(error.into()),
        }
    };

    match result {
        Ok(()) => Ok(compressed),
        Err(error) => Err(error),
    }
}

/// Formats all Vial configuration constants into a Rust source string.
///
/// Derives `VIAL_SERIAL` from the first 4 bytes of `keyboard_id` and
/// delegates byte array formatting to [`format_byte_array`].
fn format_constants(compressed: &[u8], keyboard_id: [u8; 8], vial_cfg: &str) -> String {
    let value: Value = from_str(vial_cfg).unwrap_or_default();

    let vendor_id = value
        .get("vendorId")
        .and_then(|val| val.as_str())
        .and_then(|str| u16::from_str_radix(str.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0x3434);
    let product_id = value
        .get("productId")
        .and_then(|val| val.as_str())
        .and_then(|str| u16::from_str_radix(str.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0x0B60);
    let name = value.get("name").and_then(|val| val.as_str()).unwrap_or("Keyboard");
    let id = |i: usize| keyboard_id.get(i).copied().unwrap_or(0);
    let vial_serial = format!("vial:{:02x}{:02x}{:02x}{:02x}:000001", id(0), id(1), id(2), id(3));

    let def_bytes = format_byte_array(compressed);
    let id_bytes = format_byte_array(&keyboard_id);

    format!(
        "/// XZ-compressed Vial keyboard definition, derived from `vial.json`.\n\
         pub const VIAL_KEYBOARD_DEF: &[u8] = &[{def_bytes}];\n\
         /// Unique 8-byte identifier for this keyboard, used by the Vial protocol.\n\
         pub const VIAL_KEYBOARD_ID: &[u8] = &[{id_bytes}];\n\
         /// Vial serial string derived from the first 4 bytes of [`VIAL_KEYBOARD_ID`].\n\
         pub const VIAL_SERIAL: &str = \"{vial_serial}\";\n\
         /// USB vendor ID read from `vial.json`.\n\
         pub const USB_VID: u16 = 0x{vendor_id:04X}_u16;\n\
         /// USB product ID read from `vial.json`.\n\
         pub const USB_PID: u16 = 0x{product_id:04X}_u16;\n\
         /// Keyboard product name read from `vial.json`.\n\
         pub const PRODUCT_NAME: &str = \"{name}\";\n"
    )
}

/// Formats a byte slice as a comma-separated list of Rust `u8` hex literals.
///
/// Each byte is emitted as `0xNN_u8` to satisfy the
/// `clippy::unseparated_literal_suffix` lint without requiring any allow
/// attributes in the generated file.
fn format_byte_array(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("0x{byte:02X}_u8")).collect::<Vec<_>>().join(", ")
}
