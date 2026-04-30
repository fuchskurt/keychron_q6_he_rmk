#![cfg_attr(feature = "jis_layout", expect(dead_code, reason = "JIS layout only uses some of these constants"))]

use crate::layout::{COL, ROW};

pub mod layout {
    use crate::layout::{COL, NUM_ENCODER};
    pub use rmk::types::action::{EncoderAction, KeyAction};
    use rmk::{a, df, encoder, k};

    pub const ROW0: [KeyAction; COL] = [
        k!(Escape),
        k!(F1),
        k!(F2),
        k!(F3),
        k!(F4),
        k!(F5),
        k!(F6),
        k!(F7),
        k!(F8),
        k!(F9),
        k!(F10),
        k!(F11),
        k!(F12),
        k!(AudioMute),
        k!(PrintScreen),
        k!(ScrollLock),
        k!(Pause),
        k!(F13),
        k!(F14),
        k!(F15),
        k!(F16),
    ];
    pub const ROW1: [KeyAction; COL] = [
        k!(Grave),
        k!(Kc1),
        k!(Kc2),
        k!(Kc3),
        k!(Kc4),
        k!(Kc5),
        k!(Kc6),
        k!(Kc7),
        k!(Kc8),
        k!(Kc9),
        k!(Kc0),
        k!(Minus),
        k!(Equal),
        k!(Backspace),
        k!(Insert),
        k!(Home),
        k!(PageUp),
        k!(NumLock),
        k!(KpSlash),
        k!(KpAsterisk),
        k!(KpMinus),
    ];
    pub const MAC_ROW5: [KeyAction; COL] = [
        k!(LCtrl),
        k!(LAlt),
        k!(LGui),
        a!(No),
        a!(No),
        a!(No),
        k!(Space),
        df!(0),
        df!(1),
        k!(RGui),
        k!(RAlt),
        k!(Menu),
        k!(RCtrl),
        a!(No),
        k!(Left),
        k!(Down),
        k!(Right),
        k!(Kp0),
        k!(KpDot),
        a!(No),
        a!(No),
    ];
    pub const WIN_ROW5: [KeyAction; COL] = [
        k!(LCtrl),
        k!(LGui),
        k!(LAlt),
        a!(No),
        a!(No),
        a!(No),
        k!(Space),
        df!(0),
        df!(1),
        k!(RAlt),
        k!(RGui),
        k!(Menu),
        k!(RCtrl),
        a!(No),
        k!(Left),
        k!(Down),
        k!(Right),
        k!(Kp0),
        k!(KpDot),
        a!(No),
        a!(No),
    ];
    pub const ENCODER_VOLUME: [EncoderAction; NUM_ENCODER] = [encoder!(k!(KbVolumeUp), k!(KbVolumeDown))];
}

/// Compile-time guards: if `ROW` or `COL` in `keymap.rs` ever change, the
/// hardcoded dimensions of [`crate::layout::ansi::SENSOR_POSITIONS`]
/// must be updated to match.
const _: [(); ROW] = [(); 6];
const _: [(); COL] = [(); 21];

pub mod sensor {
    use crate::layout::COL;

    // Row 0: col 13 = AudioMute mapped to the encoder button; no hall switch.
    pub const SENSOR_ROW0: [bool; COL] = [
        true, true, true, true, true, true, true, true, true, true, true, true, true, false, true, true, true, true,
        true, true, true,
    ];
    // Row 1: all 21 positions populated.
    pub const SENSOR_ROW1: [bool; COL] = [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ];
    // Row 2: all 21 positions populated.
    pub const SENSOR_ROW2: [bool; COL] = [
        true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true,
    ];
    // Row 5: cols 3–5 = Space, cols 7–8 = layer toggle, col 13 = gap, cols 19–20 =
    // KpEnter.
    pub const SENSOR_ROW5: [bool; COL] = [
        true, true, true, false, false, false, true, false, false, true, true, true, true, false, true, true, true,
        true, true, false, false,
    ];
}
pub mod led {
    pub const LED_MAPPING_ROW0: [Option<u8>; 21] = [
        Some(0),
        Some(1),
        Some(2),
        Some(3),
        Some(4),
        Some(5),
        Some(6),
        Some(7),
        Some(8),
        Some(9),
        Some(10),
        Some(11),
        Some(12),
        None,
        Some(13),
        Some(14),
        Some(15),
        Some(16),
        Some(17),
        Some(18),
        Some(19),
    ];
    pub const LED_MAPPING_ROW1: [Option<u8>; 21] = [
        Some(20),
        Some(21),
        Some(22),
        Some(23),
        Some(24),
        Some(25),
        Some(26),
        Some(27),
        Some(28),
        Some(29),
        Some(30),
        Some(31),
        Some(32),
        Some(33),
        Some(34),
        Some(35),
        Some(36),
        Some(37),
        Some(38),
        Some(39),
        Some(40),
    ];
    pub const LED_MAPPING_ROW2: [Option<u8>; 21] = [
        Some(41),
        Some(42),
        Some(43),
        Some(44),
        Some(45),
        Some(46),
        Some(47),
        Some(48),
        Some(49),
        Some(50),
        Some(51),
        Some(52),
        Some(53),
        Some(54),
        Some(55),
        Some(56),
        Some(57),
        Some(58),
        Some(59),
        Some(60),
        Some(61),
    ];
    pub const LED_MAPPING_ROW3: [Option<u8>; 21] = [
        Some(62),
        Some(63),
        Some(64),
        Some(65),
        Some(66),
        Some(67),
        Some(68),
        Some(69),
        Some(70),
        Some(71),
        Some(72),
        Some(73),
        Some(74),
        None,
        None,
        None,
        None,
        Some(75),
        Some(76),
        Some(77),
        None,
    ];
}
