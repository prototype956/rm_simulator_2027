use super::*;
use crate::robomaster::combat::{CombatRole, RobotRules, TrainingPreset};
use crate::robomaster::prelude::Team;

fn fixture() -> World {
    let mut world = World::new();
    let config: SimulationConfig = toml::from_str(include_str!("../../../../config.toml")).unwrap();
    world.insert_resource(config);
    world.insert_resource(Time::<Fixed>::default());
    world.insert_resource(ControllerState::default());
    world.insert_resource(ProjectileStatistics::default());
    world.insert_resource(ProjectileSetting(Handle::default(), Handle::default()));
    world.insert_resource(FireRequests::default());
    world.insert_resource(Messages::<FireRequestResult>::default());
    world.insert_resource(Messages::<ShotFired>::default());
    world.insert_resource(SubscribeAutoAim(std::sync::atomic::AtomicBool::new(true)));
    world
}

fn robot(world: &mut World, id: u64) -> (Entity, Entity) {
    let root = world
        .spawn((
            RobotIdentity {
                id: RobotId(id),
                team: Team::Red,
                role: CombatRole::Infantry,
            },
            RobotCombatState::new(TrainingPreset::InfantryHealthCooling.resolve().1),
            Transform::from_xyz(1.0, 2.0, 3.0),
            Position::new(Vec3::new(1.0, 2.0, 3.0)),
            Rotation::default(),
            LinearVelocity::ZERO,
        ))
        .id();
    let gimbal = world
        .spawn((Transform::from_xyz(0.0, 0.3, 0.0), ChildOf(root)))
        .id();
    let muzzle = world
        .spawn((
            InfantryLaunchOffset,
            RobotMember {
                root,
                id: RobotId(id),
            },
            Transform::from_xyz(0.0, 0.2, 0.0),
            ChildOf(gimbal),
        ))
        .id();
    (root, muzzle)
}

fn advance(world: &mut World, milliseconds: u64) {
    world
        .resource_mut::<Time<Fixed>>()
        .advance_by(Duration::from_millis(milliseconds));
    execute_fire_requests(world);
}

fn results(world: &mut World) -> Vec<FireResult> {
    world
        .resource_mut::<Messages<FireRequestResult>>()
        .drain()
        .map(|r| r.result)
        .collect()
}

#[test]
fn mixed_inputs_share_interval_and_rejections_never_catch_up() {
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1);
    robot(&mut world, 2);
    let talos = FireSource::Talos {
        command_timestamp_ns: 123,
    };
    request_fire(&mut world, RobotId(1), talos);
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    request_fire(&mut world, RobotId(1), talos);
    request_fire(&mut world, RobotId(2), FireSource::Manual);
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        0
    );
    advance(&mut world, 10);
    assert_eq!(
        results(&mut world),
        vec![
            FireResult::Fired { projectile_id: 1 },
            FireResult::Rejected(FireRejection::MechanicalInterval),
            FireResult::Rejected(FireRejection::MechanicalInterval),
            FireResult::Fired { projectile_id: 2 },
        ]
    );
    advance(&mut world, 49);
    request_fire(&mut world, RobotId(1), talos);
    execute_fire_requests(&mut world);
    assert_eq!(
        results(&mut world),
        vec![FireResult::Rejected(FireRejection::MechanicalInterval)]
    );
    advance(&mut world, 1);
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        2
    );
    request_fire(&mut world, RobotId(1), talos);
    execute_fire_requests(&mut world);
    assert_eq!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .shooter
            .actual_shots,
        2
    );
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        3
    );
    let shots: Vec<_> = world
        .resource_mut::<Messages<ShotFired>>()
        .drain()
        .collect();
    assert_eq!(
        shots.iter().map(|s| s.shot.id).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    for event in shots {
        assert!(world.get::<ProjectileShot>(event.projectile).is_some());
        assert_eq!(event.shot.muzzle_pose.translation, Vec3::new(1.0, 2.5, 3.0));
    }
}

#[test]
fn manual_hold_repeats_on_simulation_steps_and_release_does_not_bypass_interval() {
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1);
    // Keep this mechanical cadence check below the thermal ceiling introduced by module 3.
    world.get_mut::<RobotCombatState>(root).unwrap().rules =
        TrainingPreset::InfantryHealthBurst.resolve().1;
    world.entity_mut(root).insert(Controlled);
    world.resource_mut::<ControllerState>().controlled.shoot = true;
    // A 7 ms timestep does not divide the 50 ms interval; no rounding may make it shorter.
    for _ in 0..100 {
        world
            .resource_mut::<Time<Fixed>>()
            .advance_by(Duration::from_millis(7));
        collect_manual_requests(&mut world);
        execute_fire_requests(&mut world);
    }
    let shots: Vec<_> = world
        .resource_mut::<Messages<ShotFired>>()
        .drain()
        .collect();
    assert_eq!(shots.len(), 13);
    for pair in shots.windows(2) {
        assert_eq!(
            pair[1].shot.fired_at - pair[0].shot.fired_at,
            Duration::from_millis(56)
        );
    }
    let count = world.resource::<ProjectileStatistics>().bullet_launch_count;
    world.resource_mut::<ControllerState>().controlled.shoot = false;
    collect_manual_requests(&mut world);
    world.resource_mut::<ControllerState>().controlled.shoot = true;
    collect_manual_requests(&mut world);
    execute_fire_requests(&mut world);
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        count
    );
}

#[test]
fn overheating_allows_crossing_shot_then_rejects_until_zero_without_backlog() {
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1);
    let (other, _) = robot(&mut world, 2);
    for i in 0..10 {
        if i != 0 {
            advance(&mut world, 50);
        }
        request_fire(&mut world, RobotId(1), FireSource::Manual);
        execute_fire_requests(&mut world);
    }
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        90.4
    );
    assert!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .heat
            .cooling_locked
    );
    assert_eq!(
        world.get::<RobotCombatState>(other).unwrap().heat.current(),
        0.0
    );
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        10
    );
    results(&mut world);
    for source in [
        FireSource::Manual,
        FireSource::Talos {
            command_timestamp_ns: 789,
        },
    ] {
        advance(&mut world, 50);
        // At 500 ms heat is exactly 88; at 550 ms it is still 88 and still locked.
        assert_eq!(
            world.get::<RobotCombatState>(root).unwrap().heat.current(),
            88.0
        );
        request_fire(&mut world, RobotId(1), source);
        execute_fire_requests(&mut world);
        assert_eq!(
            results(&mut world),
            vec![FireResult::Rejected(FireRejection::HeatCoolingLock)]
        );
    }
    advance(&mut world, 50);
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        85.6
    );
    assert!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .heat
            .is_locked()
    );
    advance(&mut world, 3600);
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        0.0
    );
    assert!(
        !world
            .get::<RobotCombatState>(root)
            .unwrap()
            .heat
            .is_locked()
    );
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        10
    );
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    execute_fire_requests(&mut world);
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        10.0
    );
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        11
    );
}

#[test]
fn due_cooling_unlocks_before_admission_and_new_shot_is_not_cooled_twice() {
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1);
    for i in 0..10 {
        if i != 0 {
            advance(&mut world, 50);
        }
        request_fire(&mut world, RobotId(1), FireSource::Manual);
        execute_fire_requests(&mut world);
    }
    advance(&mut world, 3749); // 4199 ms: the next 100 ms tick clears the remaining 1.6 heat.
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        1.6
    );
    assert!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .heat
            .cooling_locked
    );
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    advance(&mut world, 1);
    let state = world.get::<RobotCombatState>(root).unwrap();
    assert_eq!(state.shooter.actual_shots, 11);
    assert_eq!(state.heat.current(), 10.0);
    assert!(!state.heat.is_locked());
    execute_fire_requests(&mut world);
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        10.0
    );
}

#[test]
fn round_lock_cancels_feed_and_survives_ordinary_unlock_and_input_changes() {
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1);
    world
        .get_mut::<RobotCombatState>(root)
        .unwrap()
        .shooter
        .mechanics
        .launch_delay = Duration::from_secs(1);
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    execute_fire_requests(&mut world);
    assert!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .shooter
            .pending
            .is_some()
    );
    // Inject already-latched reasons to verify admission priority/cancellation, not the threshold.
    // Exact threshold behavior is checked by heat::tests with controlled starting heat values.
    world
        .get_mut::<RobotCombatState>(root)
        .unwrap()
        .heat
        .round_locked = true;
    world
        .get_mut::<RobotCombatState>(root)
        .unwrap()
        .heat
        .cooling_locked = true;
    results(&mut world);
    advance(&mut world, 1);
    assert_eq!(
        results(&mut world),
        vec![FireResult::Rejected(FireRejection::HeatRoundLock)]
    );
    assert!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .shooter
            .pending
            .is_none()
    );
    advance(&mut world, 99);
    assert!(
        !world
            .get::<RobotCombatState>(root)
            .unwrap()
            .heat
            .cooling_locked
    );
    assert!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .heat
            .round_locked
    );
    world
        .resource::<SubscribeAutoAim>()
        .store(false, Ordering::Release);
    advance(&mut world, 1000);
    world
        .resource::<SubscribeAutoAim>()
        .store(true, Ordering::Release);
    request_fire(
        &mut world,
        RobotId(1),
        FireSource::Talos {
            command_timestamp_ns: 999,
        },
    );
    execute_fire_requests(&mut world);
    assert_eq!(
        results(&mut world),
        vec![FireResult::Rejected(FireRejection::HeatRoundLock)]
    );
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        0
    );
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        0.0
    );
}

#[test]
fn explicit_feed_delay_uses_exit_pose_and_cancels_when_external_control_is_disabled() {
    let mut world = fixture();
    let (root, muzzle) = robot(&mut world, 1);
    world
        .get_mut::<RobotCombatState>(root)
        .unwrap()
        .shooter
        .mechanics
        .launch_delay = Duration::from_millis(30);
    let talos = FireSource::Talos {
        command_timestamp_ns: 456,
    };
    request_fire(&mut world, RobotId(1), talos);
    request_fire(&mut world, RobotId(1), talos);
    execute_fire_requests(&mut world);
    assert_eq!(
        results(&mut world),
        vec![
            FireResult::Feeding {
                ready_at: Duration::from_millis(30)
            },
            FireResult::Rejected(FireRejection::FeedBusy),
        ]
    );
    world.get_mut::<Transform>(muzzle).unwrap().translation.x = 0.4;
    world.get_mut::<Transform>(muzzle).unwrap().rotation =
        Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
    world.get_mut::<LinearVelocity>(root).unwrap().0 = Vec3::Z;
    advance(&mut world, 29);
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        0
    );
    advance(&mut world, 1);
    let event = world
        .resource_mut::<Messages<ShotFired>>()
        .drain()
        .next()
        .unwrap();
    assert!((event.shot.muzzle_pose.translation - Vec3::new(1.4, 2.5, 3.0)).length() < 1e-5);
    assert!((event.shot.initial_velocity - Vec3::new(25.0, 0.0, 1.0)).length() < 1e-5);
    assert_eq!(event.shot.fired_at, Duration::from_millis(30));
    advance(&mut world, 50);
    request_fire(&mut world, RobotId(1), talos);
    execute_fire_requests(&mut world);
    world
        .resource::<SubscribeAutoAim>()
        .store(false, Ordering::Release);
    advance(&mut world, 1);
    world
        .resource::<SubscribeAutoAim>()
        .store(true, Ordering::Release);
    advance(&mut world, 100);
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        1
    );
    assert!(results(&mut world).contains(&FireResult::Rejected(
        FireRejection::ExternalControlDisabled
    )));
}

#[test]
fn missing_muzzle_and_hero_do_not_report_shots() {
    let mut world = fixture();
    let (root, muzzle) = robot(&mut world, 1);
    world.despawn(muzzle);
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    execute_fire_requests(&mut world);
    world
        .entity_mut(root)
        .insert(RobotCombatState::new(RobotRules::hero_target()));
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    execute_fire_requests(&mut world);
    assert_eq!(
        results(&mut world),
        vec![
            FireResult::Rejected(FireRejection::MissingMuzzle),
            FireResult::Rejected(FireRejection::NoShooter),
        ]
    );
    assert_eq!(
        world.resource::<ProjectileStatistics>().bullet_launch_count,
        0
    );
    assert_eq!(world.query::<&ProjectileShot>().iter(&world).count(), 0);
}

#[test]
fn illegal_speed_and_unrepresentable_mechanical_durations_are_rejected() {
    let world = fixture();
    let original = world.resource::<SimulationConfig>().projectile.clone();
    assert!(original.validate_shooter().is_ok());
    for speed in [0.0, -1.0, 25.001, f32::INFINITY, f32::NAN] {
        let mut config = original.clone();
        config.speed = speed;
        assert!(config.validate_shooter().is_err());
    }
    for interval in [0.0, -0.01, 1e-12, f64::INFINITY, f64::NAN, f64::MAX] {
        let mut config = original.clone();
        config.cooldown = interval;
        assert!(config.validate_shooter().is_err());
    }
    for delay in [-1.0, f64::INFINITY, f64::NAN, f64::MAX] {
        let mut config = original.clone();
        config.launch_delay_s = delay;
        assert!(config.validate_shooter().is_err());
    }
}

#[test]
fn finite_allowance_debits_only_actual_shots_and_cannot_bypass_heat() {
    use crate::robomaster::combat::FireAllowance;
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1);
    world.get_mut::<RobotCombatState>(root).unwrap().allowance = FireAllowance::Limited(1);
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    advance(&mut world, 1);
    let state = world.get::<RobotCombatState>(root).unwrap();
    assert_eq!(state.allowance, FireAllowance::Limited(0));
    assert_eq!(state.heat.current(), 10.0);
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    advance(&mut world, 50);
    assert_eq!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .shooter
            .actual_shots,
        1
    );
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        10.0
    );
    assert_eq!(
        results(&mut world).last(),
        Some(&FireResult::Rejected(FireRejection::AllowanceEmpty))
    );
    let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
    state.allowance = FireAllowance::Unlimited;
    state.heat.round_locked = true;
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    advance(&mut world, 50);
    assert_eq!(
        world
            .get::<RobotCombatState>(root)
            .unwrap()
            .shooter
            .actual_shots,
        1
    );
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().allowance,
        FireAllowance::Unlimited
    );
}

#[test]
fn failed_muzzle_and_pending_feed_do_not_spend_allowance() {
    use crate::robomaster::combat::FireAllowance;
    let mut world = fixture();
    let (root, muzzle) = robot(&mut world, 1);
    world.get_mut::<RobotCombatState>(root).unwrap().allowance = FireAllowance::Limited(2);
    world
        .get_mut::<RobotCombatState>(root)
        .unwrap()
        .shooter
        .mechanics
        .launch_delay = Duration::from_millis(20);
    request_fire(&mut world, RobotId(1), FireSource::Manual);
    advance(&mut world, 1);
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().allowance,
        FireAllowance::Limited(2)
    );
    world.despawn(muzzle);
    advance(&mut world, 20);
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().allowance,
        FireAllowance::Limited(2)
    );
    assert_eq!(
        world.get::<RobotCombatState>(root).unwrap().heat.current(),
        0.0
    );
}
