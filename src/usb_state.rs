//! USB host suspend state, sourced from RMK's connection-status events.
//!
//! `usb_is_active()` is a cheap synchronous atomic load — the matrix scan hot
//! loop and the backlight connection poll both call it and neither can
//! `.await`. The single [`UsbStateTask`] subscriber is the only writer: it
//! translates `ConnectionStatusChangeEvent` (published from RMK's `suspended()`
//! / `configured()` USB handlers) into the flag.

use crate::backlight::processor::{BACKLIGHT_CH, BacklightCmd};
use core::sync::atomic::{AtomicBool, Ordering};
use rmk::{
    core_traits::Runnable,
    event::{ConnectionStatusChangeEvent, EventSubscriber as _, SubscribableEvent},
    types::connection::UsbState,
};

/// `true` once the host has configured the device and has not suspended it.
///
/// Starts `false`: [`UsbStateTask::new`] subscribes before `run_all!` drives
/// USB enumeration, so the initial `Enabled → Configured` edge is captured and
/// flips this to `true`. A host-initiated suspend flips it back.
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// Mirrors USB lifecycle transitions into [`ACTIVE`]. Hand to `run_all!`.
pub struct UsbStateTask {
    /// Subscription to USB connection-status transitions; the sole writer of
    /// [`ACTIVE`].
    sub: <ConnectionStatusChangeEvent as SubscribableEvent>::Subscriber,
}

impl UsbStateTask {
    /// Subscribes immediately so the subscription is live before enumeration.
    ///
    /// Must be constructed before `run_all!`; constructing it after USB has
    /// enumerated would miss the initial `Configured` edge and latch off.
    #[must_use]
    pub fn new() -> Self { Self { sub: ConnectionStatusChangeEvent::subscriber() } }
}

impl Default for UsbStateTask {
    fn default() -> Self { Self::new() }
}

impl Runnable for UsbStateTask {
    async fn run(&mut self) -> ! {
        let mut prev = false; // matches ACTIVE's initial value
        loop {
            let active = matches!(self.sub.next_event().await.0.usb, UsbState::Configured);
            ACTIVE.store(active, Ordering::Relaxed);
            if active != prev {
                prev = active;
                BACKLIGHT_CH.sender().send(BacklightCmd::Power(active)).await;
            }
        }
    }
}

/// Returns `true` when the USB host has the device configured and awake.
#[must_use]
#[inline]
pub fn usb_is_active() -> bool { ACTIVE.load(Ordering::Relaxed) }
