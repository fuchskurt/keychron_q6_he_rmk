//! Build script for the RMK Q6 HE ANSI firmware.
//!
//! Generates Vial keyboard configuration constants from `vial.json` and
//! sets the required linker arguments for the Cortex-M4 target.

use lzma_rs::xz_compress;
use std::{env, fmt::Write, fs, path::Path};

/// Entry point for the build script.
///
/// Registers change detection for `vial.json` and `memory.x`, sets the
/// required linker arguments, and triggers Vial config generation.
fn main() {
    println!("cargo:rerun-if-changed=vial.json");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");

    generate_vial_config();
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
fn generate_vial_config() {
    let out_file = Path::new(&env::var_os("OUT_DIR").unwrap()).join("config_generated.rs");

    let vial_cfg = read_vial_json();
    let compressed = compress_vial_cfg(&vial_cfg);
    let keyboard_id: [u8; 8] = [0x36, 0x3B, 0x06, 0xF5, 0x13, 0x9F, 0xE4, 0x46];

    fs::write(out_file, format_constants(&compressed, &keyboard_id)).unwrap();
}

/// Reads and minifies `vial.json` from the crate root.
///
/// Parses the JSON and re-stringifies it to strip whitespace before
/// compression. Prints a diagnostic message to stderr if the file cannot
/// be opened, and returns an empty string so compression can still proceed.
fn read_vial_json() -> String {
    fs::read_to_string("vial.json").map(|content| json::stringify(json::parse(&content).unwrap())).unwrap_or_else(|e| {
        eprintln!("Cannot find vial.json: {e}");
        String::new()
    })
}

/// Compresses a Vial config string using XZ at compression level 6.
///
/// Returns the compressed bytes as a `Vec<u8>` ready to be embedded as a
/// constant in the generated source file.
fn compress_vial_cfg(vial_cfg: &str) -> Vec<u8> {
    let mut compressed = Vec::new();
    xz_compress(&mut vial_cfg.as_bytes(), &mut compressed).unwrap();
    compressed
}

/// Formats all Vial configuration constants into a Rust source string.
///
/// Derives `VIAL_SERIAL` from the first 4 bytes of `keyboard_id` and
/// delegates byte array formatting to [`format_byte_array`].
fn format_constants(compressed: &[u8], keyboard_id: &[u8; 8]) -> String {
    let id = |i: usize| keyboard_id.get(i).copied().unwrap_or(0);
    let vial_serial = format!("vial:{:02x}{:02x}{:02x}{:02x}:000001", id(0), id(1), id(2), id(3),);

    let def_bytes = format_byte_array(compressed);
    let id_bytes = format_byte_array(keyboard_id);

    format!(
        "/// XZ-compressed Vial keyboard definition, derived from `vial.json`.\n\
         pub const VIAL_KEYBOARD_DEF: &[u8] = &[{def_bytes}];\n\
         /// Unique 8-byte identifier for this keyboard, used by the Vial protocol.\n\
         pub const VIAL_KEYBOARD_ID: &[u8] = &[{id_bytes}];\n\
         /// Vial serial string derived from the first 4 bytes of [`VIAL_KEYBOARD_ID`].\n\
         pub const VIAL_SERIAL: &str = \"{vial_serial}\";\n"
    )
}

/// Formats a byte slice as a comma-separated list of Rust `u8` hex literals.
///
/// Each byte is emitted as `0xNN_u8` to satisfy the
/// `clippy::unseparated_literal_suffix` lint without requiring any allow
/// attributes in the generated file.
fn format_byte_array(bytes: &[u8]) -> String {
    bytes.iter().enumerate().fold(String::new(), |mut acc, (i, b)| {
        if i > 0 {
            acc.push_str(", ");
        }
        write!(acc, "0x{b:02X}_u8").unwrap();
        acc
    })
}
