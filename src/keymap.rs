use rmk::{
    a,
    encoder,
    k,
    layer,
    types::action::{EncoderAction, KeyAction},
};

pub(crate) const COL: usize = 21;
pub(crate) const ROW: usize = 6;
pub(crate) const NUM_LAYER: usize = 1;

pub(crate) const NUM_ENCODER: usize = 1;

#[rustfmt::skip]
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        layer!(
        // Layer 0: WIN_BASE (QMK LAYOUT_ansi_108 -> RMK 6x21)
        [
            // Row 0 (21 keys):
            // KC_ESC, KC_F1..KC_F12, KC_MUTE, KC_PSCR, KC_CTANA, RGB_MOD, KC_F13..KC_F16
            [
                k!(Escape), k!(F1),  k!(F2),  k!(F3),  k!(F4),  k!(F5),  k!(F6),
                k!(F7),     k!(F8),  k!(F9),  k!(F10), k!(F11), k!(F12),
                k!(AudioMute), k!(PrintScreen),
                a!(No), // KC_CTANA (replace with your RMK/custom action if available)
                a!(No), // RGB_MOD  (replace with your RMK RGB action)
                k!(F13), k!(F14), k!(F15), k!(F16)
            ],

            // Row 1 (21 keys):
            // KC_GRV, 1..0, -, =, BSPC, INS, HOME, PGUP, NUM, /, *, -
            [
                k!(Grave), k!(Kc1), k!(Kc2), k!(Kc3), k!(Kc4), k!(Kc5), k!(Kc6),
                k!(Kc7),   k!(Kc8), k!(Kc9), k!(Kc0),
                k!(Minus), k!(Equal), k!(Backspace),
                k!(Insert), k!(Home), k!(PageUp),
                k!(NumLock), k!(KpSlash), k!(KpAsterisk), k!(KpMinus)
            ],

            // Row 2 (21 keys):
            // KC_TAB, Q..P, [, ], \, DEL, END, PGDN, P7, P8, P9, PPLS
            [
                k!(Tab), k!(Q), k!(W), k!(E), k!(R), k!(T), k!(Y),
                k!(U),   k!(I), k!(O), k!(P),
                k!(LeftBracket), k!(RightBracket), k!(Backslash),
                k!(Delete), k!(End), k!(PageDown),
                k!(Kp7), k!(Kp8), k!(Kp9), k!(KpPlus)
            ],

            // Row 3 (21 keys):
            // KC_CAPS, A..', (gap), ENT, (gaps...), P4, P5, P6
            // In QMK this row has blanks where there is no switch; we represent them as No.
            [
                k!(CapsLock), k!(A), k!(S), k!(D), k!(F), k!(G), k!(H),
                k!(J), k!(K), k!(L), k!(Semicolon), k!(Quote),
                a!(No), // (QMK has an empty position here)
                k!(Enter),
                a!(No), a!(No), a!(No), a!(No), // QMK "holes"
                k!(Kp4), k!(Kp5), k!(Kp6)
            ],

            // Row 4 (21 keys):
            // KC_LSFT, (gap), Z.. /, (gap), RSFT, (gap), UP, (gap), P1 P2 P3 PENT
            [
                k!(LShift),
                a!(No), // (QMK gap)
                k!(Z), k!(X), k!(C), k!(V), k!(B), k!(N), k!(M),
                k!(Comma), k!(Dot), k!(Slash),
                a!(No), // (QMK gap)
                k!(RShift),
                a!(No), // (QMK gap)
                k!(Up),
                a!(No), // (QMK gap)
                k!(Kp1), k!(Kp2), k!(Kp3), k!(KpEnter)
            ],

            // Row 5 (21 keys):
            // KC_LCTL, KC_LWIN, KC_LALT, (gaps...), SPC, (gaps...), RALT, RWIN, FN_WIN, RCTL, LEFT DOWN RIGHT, P0 (gap) PDOT
            [
                k!(LCtrl), k!(LGui), k!(LAlt),
                a!(No), a!(No), a!(No), // left-space "holes"
                k!(Space),
                a!(No), a!(No), a!(No), // right-space "holes"
                k!(RAlt), k!(RGui),
                a!(No), // FN_WIN (replace with your RMK layer switch, e.g. mo!(WIN_FN))
                k!(RCtrl),
                k!(Left), k!(Down), k!(Right),
                k!(Kp0),
                a!(No),
                a!(No), // (QMK gap between P0 and PDOT)
                k!(KpDot)
            ]
        ]),
    ]
}

#[rustfmt::skip]
pub const fn get_default_encoder_map() -> [[EncoderAction; NUM_ENCODER]; NUM_LAYER] {
    [
        // Layer 0
        [
            // Encoder 0: (Clockwise, Counter-Clockwise)
            encoder!(k!(KbVolumeUp), k!(KbVolumeDown)),
        ],
    ]
}
