//! USB host connection state read directly from the OTG FS hardware
//! registers, shared by the backlight and matrix scan tasks.
//!
//! Reading the registers directly (rather than relying on embassy-usb
//! events) means suspend and disconnect are detected even when the USB
//! handler does not fire, and lets independent tasks poll the same state
//! without coordination.

use embassy_stm32::pac::USB_OTG_FS;

/// Returns `true` when the USB host is present and the bus is not suspended.
#[must_use]
#[inline]
pub fn usb_is_active() -> bool { usb_vbus_present() && !usb_is_suspended() }

/// Returns `true` when the OTG FS hardware reports the USB bus is suspended.
///
/// Reads `OTG_FS.DSTS.SUSPSTS` directly.
#[must_use]
#[inline]
fn usb_is_suspended() -> bool { USB_OTG_FS.dsts().read().suspsts() }

/// Returns `true` when VBUS is present (cable plugged in, host powered).
///
/// Reads `OTG_FS.GOTGCTL.BSVLD` directly.
#[must_use]
#[inline]
fn usb_vbus_present() -> bool { USB_OTG_FS.gotgctl().read().bsvld() }
