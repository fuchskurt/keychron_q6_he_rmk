#![expect(clippy::min_ident_chars, reason = "rmk macro names a!/k! are fixed upstream and cannot be renamed")]

use crate::layout::{COL, NUM_ENCODER};
pub use rmk::types::action::{EncoderAction, KeyAction};
use rmk::{a as act, df, encoder, k as key};

/// Default key actions for matrix row 0 (shared across layers).
pub const ROW0: [KeyAction; COL] = [
    key!(Escape),
    key!(F1),
    key!(F2),
    key!(F3),
    key!(F4),
    key!(F5),
    key!(F6),
    key!(F7),
    key!(F8),
    key!(F9),
    key!(F10),
    key!(F11),
    key!(F12),
    key!(AudioMute),
    key!(PrintScreen),
    key!(ScrollLock),
    key!(Pause),
    key!(F13),
    key!(F14),
    key!(F15),
    key!(F16),
];
/// Default key actions for matrix row 1 (shared across layers).
pub const ROW1: [KeyAction; COL] = [
    key!(Grave),
    key!(Kc1),
    key!(Kc2),
    key!(Kc3),
    key!(Kc4),
    key!(Kc5),
    key!(Kc6),
    key!(Kc7),
    key!(Kc8),
    key!(Kc9),
    key!(Kc0),
    key!(Minus),
    key!(Equal),
    key!(Backspace),
    key!(Insert),
    key!(Home),
    key!(PageUp),
    key!(NumLock),
    key!(KpSlash),
    key!(KpAsterisk),
    key!(KpMinus),
];
/// Key actions for matrix row 5 on the Mac base layer.
pub const MAC_ROW5: [KeyAction; COL] = [
    key!(LCtrl),
    key!(LAlt),
    key!(LGui),
    act!(No),
    act!(No),
    act!(No),
    key!(Space),
    df!(0),
    df!(1),
    key!(RGui),
    key!(RAlt),
    key!(Menu),
    key!(RCtrl),
    act!(No),
    key!(Left),
    key!(Down),
    key!(Right),
    key!(Kp0),
    key!(KpDot),
    act!(No),
    act!(No),
];
/// Key actions for matrix row 5 on the Windows base layer.
pub const WIN_ROW5: [KeyAction; COL] = [
    key!(LCtrl),
    key!(LGui),
    key!(LAlt),
    act!(No),
    act!(No),
    act!(No),
    key!(Space),
    df!(0),
    df!(1),
    key!(RAlt),
    key!(RGui),
    key!(Menu),
    key!(RCtrl),
    act!(No),
    key!(Left),
    key!(Down),
    key!(Right),
    key!(Kp0),
    key!(KpDot),
    act!(No),
    act!(No),
];
/// Default rotary-encoder action map: volume up/down.
pub const ENCODER_VOLUME: [EncoderAction; NUM_ENCODER] = [encoder!(key!(KbVolumeUp), key!(KbVolumeDown))];
