use super::*;
use crate::robomaster::combat::TrainingPreset;

#[test]
fn ordinary_limit_is_strict_and_unlock_requires_exact_zero() {
    let mut heat = HeatState {
        current_tenths: 780,
        ..default()
    };
    heat.add_shot(88);
    assert_eq!(heat.current(), 88.0);
    assert!(!heat.is_locked());
    heat.add_shot(88);
    assert_eq!(heat.current(), 98.0);
    assert!(heat.cooling_locked);
    assert!(!heat.round_locked);
    for _ in 0..5 {
        heat.cool_once(24);
    }
    assert_eq!(heat.current(), 86.0);
    assert!(
        heat.cooling_locked,
        "below the upper limit must remain locked"
    );
    for _ in 0..35 {
        heat.cool_once(24);
    }
    assert_eq!(heat.current(), 2.0);
    assert!(heat.cooling_locked);
    heat.cool_once(24);
    assert_eq!(heat.current(), 0.0);
    assert!(!heat.is_locked());
    heat.cool_once(24);
    assert_eq!(heat.current(), 0.0);
}

#[test]
fn round_lock_uses_flowchart_equality_and_survives_zero_heat() {
    for preset in [
        TrainingPreset::InfantryHealthCooling,
        TrainingPreset::InfantryHealthBurst,
        TrainingPreset::Sentry,
    ] {
        let rules = preset.resolve().1;
        let boundary = (u64::from(rules.heat_limit) + 100) * 10;
        // Controlled initial states: ordinary immediate locking prevents natural arrival here.
        for final_heat in [boundary - 1, boundary, boundary + 1] {
            let mut heat = HeatState {
                current_tenths: final_heat - SHOT_HEAT_TENTHS,
                ..default()
            };
            heat.add_shot(rules.heat_limit);
            assert!(heat.cooling_locked);
            assert_eq!(heat.round_locked, final_heat >= boundary);
            heat.cool_until(
                Duration::from_secs(30),
                rules.cooling_per_second,
                |_, _, _| {},
            );
            assert_eq!(heat.current(), 0.0);
            assert!(!heat.cooling_locked);
            assert_eq!(heat.round_locked, final_heat >= boundary);
            assert_eq!(heat.is_locked(), final_heat >= boundary);
        }
    }
}

#[test]
fn cooling_is_discrete_at_deadlines_and_duplicate_time_never_cools_twice() {
    let mut heat = HeatState {
        current_tenths: 1000,
        ..default()
    };
    let mut deadlines = Vec::new();
    heat.cool_until(Duration::from_millis(99), 24, |at, _, _| deadlines.push(at));
    assert_eq!(heat.current(), 100.0);
    heat.cool_until(Duration::from_millis(100), 24, |at, _, _| {
        deadlines.push(at)
    });
    assert_eq!(heat.current(), 97.6);
    heat.cool_until(Duration::from_millis(100), 24, |at, _, _| {
        deadlines.push(at)
    });
    assert_eq!(heat.current(), 97.6);
    heat.cool_until(Duration::from_millis(650), 24, |at, _, _| {
        deadlines.push(at)
    });
    assert_eq!(heat.current(), 85.6);
    assert_eq!(
        deadlines,
        (1..=6)
            .map(|i| Duration::from_millis(i * 100))
            .collect::<Vec<_>>()
    );
}

#[test]
fn cooling_matches_across_frame_rates_and_catch_up_sizes() {
    for rate in [14, 24, 30] {
        for fps in [1, 7, 30, 60, 144] {
            let mut heat = HeatState {
                current_tenths: 1000,
                cooling_locked: true,
                ..default()
            };
            // Every grouping ends at exactly one second, including rates not dividing 10 Hz.
            for frame in 1..=fps {
                let now = Duration::from_nanos(frame * 1_000_000_000 / fps);
                heat.cool_until(now, rate, |_, _, _| {});
            }
            assert_eq!(heat.current_tenths, 1000 - u64::from(rate) * 10);
            assert_eq!(heat.last_cooling_at, Duration::from_secs(1));
            assert!(heat.cooling_locked);
            heat.cool_until(Duration::from_secs(10), rate, |_, _, _| {});
            assert_eq!(heat.current_tenths, 0);
            assert!(!heat.is_locked());
        }
    }
}

#[test]
fn cold_ticks_do_not_accumulate_credit_and_new_scene_has_its_own_epoch() {
    let mut heat = HeatState::new(Duration::from_millis(350));
    heat.cool_until(Duration::from_secs(5), 24, |_, _, _| {});
    assert_eq!(heat.last_cooling_at, Duration::from_millis(4950));
    heat.add_shot(88);
    heat.cool_until(Duration::from_secs(5), 24, |_, _, _| {});
    assert_eq!(heat.current(), 10.0);
    heat.cool_until(Duration::from_millis(5050), 24, |_, _, _| {});
    assert_eq!(heat.current(), 7.6);
    let fresh = HeatState::new(Duration::from_millis(5050));
    assert_eq!(fresh.current(), 0.0);
    assert!(!fresh.is_locked());
}

#[test]
fn thermal_lock_summary_counts_and_closes_simulation_duration() {
    let mut heat = HeatState::new(Duration::ZERO);
    let before = heat.snapshot();
    heat.add_shot(1);
    heat.track_locks(Duration::from_secs(1), before);
    assert_eq!(heat.lock_count, 1);
    assert_eq!(
        heat.locked_duration(Duration::from_secs(3)),
        Duration::from_secs(2)
    );
    heat.on_death(Duration::from_secs(4));
    assert_eq!(
        heat.locked_duration(Duration::from_secs(9)),
        Duration::from_secs(3)
    );
    let heat = HeatState::new(Duration::from_secs(9));
    assert_eq!(heat.lock_count, 0);
    assert_eq!(
        heat.locked_duration(Duration::from_secs(10)),
        Duration::ZERO
    );
}
