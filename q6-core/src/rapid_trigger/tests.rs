use super::step;

const ACT: u8 = 10;
const PRESS_SENS: u8 = 5;
const RELEASE_SENS: u8 = 3;
const TROUGH_FLOOR: u8 = 8;

fn run(pressed: &mut bool, extremum: &mut u8, travels: &[u8]) -> Option<bool> {
    let mut last = None;
    for &travel in travels {
        last = step(pressed, extremum, travel, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    }
    last
}

#[test]
fn below_actuation_never_presses() {
    let (mut pressed, mut extremum) = (false, u8::MAX);
    for travel in 0..ACT {
        let result = step(&mut pressed, &mut extremum, travel, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
        assert_eq!(result, None);
        assert!(!pressed);
    }
}

#[test]
fn first_press_fires_at_actuation_threshold() {
    // From a freshly released state (extremum=0 after a sub-actuation dip),
    // press fires the first time travel meets both `extremum + press_sens`
    // and `act_threshold`. With extremum=0 and PRESS_SENS=5, the binding
    // constraint is the actuation threshold (10).
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0]);
    assert_eq!(extremum, 0);

    let just_under =
        step(&mut pressed, &mut extremum, ACT.saturating_sub(1), ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(just_under, None);
    assert!(!pressed);

    let at_threshold = step(&mut pressed, &mut extremum, ACT, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(at_threshold, Some(true));
    assert!(pressed);
}

#[test]
fn press_transition_resets_extremum_to_current_travel() {
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0]);
    let transition = step(&mut pressed, &mut extremum, 30, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(transition, Some(true));
    assert_eq!(extremum, 30, "extremum must reset to the transition travel value");
}

#[test]
fn released_via_rapid_trigger_drop_emits_release() {
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0, 30]);
    assert!(pressed);
    // Push deeper to raise the tracked peak.
    step(&mut pressed, &mut extremum, 40, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(extremum, 40);
    // A drop strictly within sensitivity_release keeps the press.
    // peak=40, sensitivity_release=3 → release fires when travel <= 37.
    let small_dip = step(&mut pressed, &mut extremum, 38, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(small_dip, None);
    // A drop past sensitivity_release fires a release.
    let release = step(&mut pressed, &mut extremum, 36, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(release, Some(false));
    assert!(!pressed);
    assert_eq!(extremum, 36);
}

#[test]
fn below_actuation_forces_release_regardless_of_peak() {
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0, 30, 40]);
    assert!(pressed);
    // Driving below act_threshold must release even though the peak is far
    // above sensitivity_release.
    let release = step(&mut pressed, &mut extremum, 0, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(release, Some(false));
    assert!(!pressed);
}

#[test]
fn extremum_tracks_trough_downward_in_released_state() {
    // Each new lower travel value lowers the tracked trough while released.
    let (mut pressed, mut extremum) = (false, 50);
    let r1 = step(&mut pressed, &mut extremum, 40, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(r1, None);
    assert_eq!(extremum, 40);
    let r2 = step(&mut pressed, &mut extremum, 15, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(r2, None);
    assert_eq!(extremum, 15);
}

#[test]
fn trough_floor_caps_extremum_when_driven_below_actuation() {
    // From a released trough sitting above the floor, driving travel below
    // the actuation threshold caps the tracked trough at `trough_floor`
    // (rather than leaving it at the higher previous value), so the next
    // re-press fires at a known baseline.
    let (mut pressed, mut extremum) = (false, 20);
    step(&mut pressed, &mut extremum, ACT.saturating_sub(1), ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(extremum, TROUGH_FLOOR);
}

#[test]
fn rapid_re_press_after_rt_release_does_not_require_full_release() {
    // Press deeply, RT-release on a moderate drop, then a small rise should
    // refire without needing to drop below the actuation floor first.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0, 30, 40]);
    assert!(pressed);
    // RT-release: drop past sensitivity_release.
    let release = step(&mut pressed, &mut extremum, 36, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(release, Some(false));
    assert_eq!(extremum, 36);
    // From trough=36, a rise of sensitivity_press counts re-fires the press
    // without first dropping below the actuation floor.
    let repress = step(
        &mut pressed,
        &mut extremum,
        36_u8.saturating_add(PRESS_SENS),
        ACT,
        PRESS_SENS,
        RELEASE_SENS,
        TROUGH_FLOOR,
    );
    assert_eq!(repress, Some(true));
    assert!(pressed);
}

#[test]
fn small_dip_within_release_band_keeps_press_and_preserves_peak() {
    // While pressed the extremum tracks the peak upward, and a dip strictly
    // within `sensitivity_release` of the peak must neither release nor
    // lower the tracked peak.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0, 30]);
    step(&mut pressed, &mut extremum, 50, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(extremum, 50);
    let dip = step(&mut pressed, &mut extremum, 48, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(dip, None);
    // The peak (50) is the max(50, 48); a small dip does not lower it.
    assert_eq!(extremum, 50);
}

#[test]
fn zero_sensitivity_press_fires_at_any_rise_past_actuation() {
    // With sensitivity_press = 0 the only press constraint is the actuation
    // threshold; any reading at or above it from a sub-actuation state fires.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    step(&mut pressed, &mut extremum, 0, ACT, 0, RELEASE_SENS, TROUGH_FLOOR);
    let press = step(&mut pressed, &mut extremum, ACT, ACT, 0, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(press, Some(true));
}

#[test]
fn zero_sensitivity_release_fires_on_any_drop_below_peak() {
    // With sensitivity_release = 0, while pressed, any travel strictly
    // below the peak triggers a release.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0, 30, 50]);
    let release = step(&mut pressed, &mut extremum, 49, ACT, PRESS_SENS, 0, TROUGH_FLOOR);
    assert_eq!(release, Some(false));
}

#[test]
fn act_threshold_above_full_travel_unit_never_fires_press() {
    // A keyboard caller sets act_threshold within the quantised travel range
    // (0..=FULL_TRAVEL_UNIT=40); raising it above the realistic range stops
    // any press from firing.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    for travel in [0_u8, 10, 20, 30, 40] {
        let result = step(&mut pressed, &mut extremum, travel, 50, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
        assert_eq!(result, None);
        assert!(!pressed);
    }
}

#[test]
fn act_threshold_zero_fires_on_first_above_extremum_plus_press_sens() {
    // act_threshold = 0 disables the actuation floor; only the rapid-trigger
    // delta (`extremum + sensitivity_press`) gates a press.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    // Settle the trough at 5.
    step(&mut pressed, &mut extremum, 5, 0, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(extremum, 5);
    // Press fires at trough + sensitivity_press = 10.
    let press = step(&mut pressed, &mut extremum, 10, 0, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(press, Some(true));
}

#[test]
fn full_press_release_cycle_round_trips_state() {
    // A full press → hold → release cycle ends in the same pressed=false
    // state it started in, but with extremum reset to the last travel value
    // seen at the release transition (not back to u8::MAX).
    let (mut pressed, mut extremum) = (false, u8::MAX);
    // Settle the trough at the floor so the first crossing fires a press.
    run(&mut pressed, &mut extremum, &[0]);
    let press = step(&mut pressed, &mut extremum, 30, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(press, Some(true));
    step(&mut pressed, &mut extremum, 40, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    step(&mut pressed, &mut extremum, 38, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    let release = step(&mut pressed, &mut extremum, 0, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(release, Some(false));
    assert!(!pressed);
    // Extremum was reset on the release transition.
    assert_eq!(extremum, 0);
}

#[test]
fn double_tap_emits_two_distinct_press_events() {
    // The release transition resets `extremum` to the current travel value
    // (here 0), so the second tap fires from the same baseline as the first.
    let (mut pressed, mut extremum) = (false, u8::MAX);
    run(&mut pressed, &mut extremum, &[0]);
    let first = step(&mut pressed, &mut extremum, 30, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(first, Some(true));
    let release = step(&mut pressed, &mut extremum, 0, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(release, Some(false));
    let second = step(&mut pressed, &mut extremum, 30, ACT, PRESS_SENS, RELEASE_SENS, TROUGH_FLOOR);
    assert_eq!(second, Some(true));
}
