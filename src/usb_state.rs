//! USB suspend/resume fan-out, sourced from RMK's connection-status events.
//!
//! `UsbStateTask` subscribes once to `ConnectionStatusChangeEvent` and, on
//! each USB lifecycle edge, pushes the active/suspended transition to the
//! backlight (via `BACKLIGHT_CH`) and the matrix (via `USB_ACTIVE`). Nothing
//! polls; both consumers react to edges.

use crate::backlight::processor::{BACKLIGHT_CH, BacklightCmd};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use rmk::{
    core_traits::Runnable,
    event::{ConnectionStatusChangeEvent, EventSubscriber as _, SubscribableEvent},
    types::connection::UsbState,
};

/// Receiver count for [`USB_ACTIVE`] (matrix scan task only).
const USB_ACTIVE_RECEIVERS: usize = 1;

/// Latest USB activity edge: `true` = host configured and awake, `false` =
/// suspended.
///
/// A retained-value [`Watch`] rather than a pub/sub channel, so a receiver
/// created after the initial `Configured` edge still observes it.
pub static USB_ACTIVE: Watch<CriticalSectionRawMutex, bool, USB_ACTIVE_RECEIVERS> = Watch::new();

/// Receiver handle for [`USB_ACTIVE`].
pub type UsbReceiver = Receiver<'static, CriticalSectionRawMutex, bool, USB_ACTIVE_RECEIVERS>;

/// Event subscriber fanning USB transitions out to the backlight and matrix.
///
/// Hand to `run_all!`; construct before it so the subscription is live before
/// USB enumeration publishes the first edge.
pub struct UsbStateTask {
    /// Subscription to USB connection-status transitions; the sole producer of
    /// edges.
    sub: <ConnectionStatusChangeEvent as SubscribableEvent>::Subscriber,
}

impl UsbStateTask {
    /// Subscribe immediately so no enumeration edge is missed.
    #[must_use]
    pub fn new() -> Self { Self { sub: ConnectionStatusChangeEvent::subscriber() } }
}

impl Default for UsbStateTask {
    fn default() -> Self { Self::new() }
}

impl Runnable for UsbStateTask {
    async fn run(&mut self) -> ! {
        let tx = USB_ACTIVE.sender();
        let mut prev = false;
        loop {
            let active = matches!(self.sub.next_event().await.0.usb, UsbState::Configured);
            if active != prev {
                prev = active;
                tx.send(active);
                BACKLIGHT_CH.sender().send(BacklightCmd::Power(active)).await;
            }
        }
    }
}

/// Wait until the host is configured and awake.
///
/// Resolves immediately if the retained value is already active, otherwise on
/// the first activating transition.
pub async fn wait_active(rx: &mut UsbReceiver) {
    if rx.try_get() == Some(true) {
        return;
    }
    loop {
        if rx.changed().await {
            return;
        }
    }
}
