//! Default keymap definitions and sizes.

#![expect(clippy::min_ident_chars, reason = "RMK-side implementation")]
use rmk::{
    a,
    df,
    encoder,
    k,
    layer,
    types::action::{EncoderAction, KeyAction},
};

/// Number of columns in the key matrix.
pub const COL: usize = 21;
/// Number of rows in the key matrix.
pub const ROW: usize = 6;
/// Number of supported layers.
pub const NUM_LAYER: usize = 2;

/// Number of rotary encoders.
pub const NUM_ENCODER: usize = 1;

#[rustfmt::skip]
/// Return the default keymap for all layers.
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        layer!(
            // Layer 0: MAC_BASE
        [
            [k!(Escape),   k!(F1),   k!(F2),    k!(F3),  k!(F4),  k!(F5),  k!(F6),    k!(F7),  k!(F8),  k!(F9),    k!(F10),       k!(F11),         k!(F12),          k!(AudioMute), k!(PrintScreen), k!(ScrollLock), k!(Pause),  k!(F13),     k!(F14),     k!(F15),        k!(F16)],
            [k!(Grave),    k!(Kc1),  k!(Kc2),   k!(Kc3), k!(Kc4), k!(Kc5), k!(Kc6),   k!(Kc7), k!(Kc8), k!(Kc9),   k!(Kc0),       k!(Minus),       k!(Equal),        k!(Backspace), k!(Insert),      k!(Home),   k!(PageUp),   k!(NumLock), k!(KpSlash), k!(KpAsterisk), k!(KpMinus)],
            [k!(Tab),      k!(Q),    k!(W),     k!(E),   k!(R),   k!(T),   k!(Y),     k!(U),   k!(I),   k!(O),     k!(P),         k!(LeftBracket), k!(RightBracket), k!(Backslash), k!(Delete),      k!(End),    k!(PageDown), k!(Kp7),     k!(Kp8),     k!(Kp9),        k!(KpPlus)],
            [k!(CapsLock), k!(A),    k!(S),     k!(D),   k!(F),   k!(G),   k!(H),     k!(J),   k!(K),   k!(L),     k!(Semicolon), k!(Quote),       k!(Enter),        a!(No),        a!(No),          a!(No),     a!(No),       k!(Kp4),     k!(Kp5),     k!(Kp6),        a!(No)],
            [k!(LShift),   a!(No),   k!(Z),     k!(X),   k!(C),   k!(V),   k!(B),     k!(N),   k!(M),   k!(Comma), k!(Dot),       k!(Slash),       a!(No),           k!(RShift),    a!(No),          k!(Up),     a!(No),       k!(Kp1),     k!(Kp2),     k!(Kp3),        k!(KpEnter)],
            [k!(LCtrl),    k!(LAlt), k!(LGui),  a!(No),  a!(No),  a!(No),  k!(Space), df!(0),  df!(1),  k!(RGui),  k!(RAlt),      k!(Menu),        k!(RCtrl),        a!(No),        k!(Left),        k!(Down),   k!(Right),    k!(Kp0),     k!(KpDot),   a!(No),         a!(No)]
        ]),
        layer!(
            // Layer 1: WIN_BASE
        [
            [k!(Escape),   k!(F1),   k!(F2),    k!(F3),  k!(F4),  k!(F5),  k!(F6),    k!(F7),  k!(F8),  k!(F9),    k!(F10),       k!(F11),         k!(F12),          k!(AudioMute), k!(PrintScreen), k!(ScrollLock), k!(Pause),  k!(F13),     k!(F14),     k!(F15),        k!(F16)],
            [k!(Grave),    k!(Kc1),  k!(Kc2),   k!(Kc3), k!(Kc4), k!(Kc5), k!(Kc6),   k!(Kc7), k!(Kc8), k!(Kc9),   k!(Kc0),       k!(Minus),       k!(Equal),        k!(Backspace), k!(Insert),      k!(Home),   k!(PageUp),   k!(NumLock), k!(KpSlash), k!(KpAsterisk), k!(KpMinus)],
            [k!(Tab),      k!(Q),    k!(W),     k!(E),   k!(R),   k!(T),   k!(Y),     k!(U),   k!(I),   k!(O),     k!(P),         k!(LeftBracket), k!(RightBracket), k!(Backslash), k!(Delete),      k!(End),    k!(PageDown), k!(Kp7),     k!(Kp8),     k!(Kp9),        k!(KpPlus)],
            [k!(CapsLock), k!(A),    k!(S),     k!(D),   k!(F),   k!(G),   k!(H),     k!(J),   k!(K),   k!(L),     k!(Semicolon), k!(Quote),       k!(Enter),        a!(No),        a!(No),          a!(No),     a!(No),       k!(Kp4),     k!(Kp5),     k!(Kp6),        a!(No)],
            [k!(LShift),   a!(No),   k!(Z),     k!(X),   k!(C),   k!(V),   k!(B),     k!(N),   k!(M),   k!(Comma), k!(Dot),       k!(Slash),       a!(No),           k!(RShift),    a!(No),          k!(Up),     a!(No),       k!(Kp1),     k!(Kp2),     k!(Kp3),        k!(KpEnter)],
            [k!(LCtrl),    k!(LGui), k!(LAlt),  a!(No),  a!(No),  a!(No),  k!(Space), df!(0),  df!(1),  k!(RAlt),  k!(RGui),      k!(Menu),        k!(RCtrl),        a!(No),        k!(Left),        k!(Down),   k!(Right),    k!(Kp0),     k!(KpDot),   a!(No),         a!(No)]
        ]),
    ]
}

#[rustfmt::skip]
/// Return the default encoder action map for each layer.
pub const fn get_default_encoder_map() -> [[EncoderAction; NUM_ENCODER]; NUM_LAYER] {
    [
        // Layer 0: MAC_BASE
        [
            // Encoder 0: (Clockwise, Counter-Clockwise)
            encoder!(k!(KbVolumeUp), k!(KbVolumeDown)),
        ],
        // Layer 1:  WIN_BASE
        [
            // Encoder 0: (Clockwise, Counter-Clockwise)
            encoder!(k!(KbVolumeUp), k!(KbVolumeDown)),
        ],
    ]
}
