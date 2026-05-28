//! Dynamic rapid-trigger press/release detection.
//!
//! Tracks the per-key travel extremum (peak while pressed, trough while
//! released) and decides whether the key just toggled state. The state is
//! intentionally passed in as mutable references rather than owned in a
//! struct here so the firmware's `repr(C)` `KeyEntry` layout stays
//! untouched.

#[cfg(test)]
mod tests;

/// Apply rapid-trigger logic for a new travel reading and report a press
/// state transition if one occurred.
///
/// `pressed` and `extremum` carry the per-key state across calls. `extremum`
/// is updated in place every call; `pressed` is updated when a transition is
/// detected, in which case `extremum` is also reset to `new_travel` so the
/// next direction starts fresh from the transition point.
///
/// Returns `None` when no transition occurred and `Some(now_pressed)` on a
/// press or release transition.
///
/// `trough_floor` clamps the released-side extremum so a key driven below
/// `act_threshold` re-fires cleanly at the actuation floor without requiring
/// an extra `sensitivity_press` delta above it.
#[inline]
#[optimize(speed)]
pub fn step(
    pressed: &mut bool,
    extremum: &mut u8,
    new_travel: u8,
    act_threshold: u8,
    sensitivity_press: u8,
    sensitivity_release: u8,
    trough_floor: u8,
) -> Option<bool> {
    let below_act = new_travel < act_threshold;
    let now_pressed = if *pressed {
        // Track the peak while held; release when travel drops at least
        // `sensitivity_release` below it. The actuation floor is a hard
        // lower bound that forces an immediate release.
        *extremum = (*extremum).max(new_travel);
        if below_act { false } else { new_travel > (*extremum).saturating_sub(sensitivity_release) }
    } else {
        // Track the trough while released; re-press when travel climbs at
        // least `sensitivity_press` above it AND exceeds the actuation
        // floor. The trough is not required to have dipped below the floor
        // first, so a finger hovering mid-travel after an RT-release can
        // re-fire immediately.
        *extremum = (*extremum).min(new_travel);
        if below_act {
            *extremum = (*extremum).min(trough_floor);
            false
        } else {
            new_travel >= (*extremum).saturating_add(sensitivity_press)
        }
    };
    let changed = now_pressed != *pressed;
    *pressed = now_pressed;
    if changed {
        // Reset extremum so the next direction starts fresh from the
        // transition point.
        *extremum = new_travel;
    }
    changed.then_some(now_pressed)
}
