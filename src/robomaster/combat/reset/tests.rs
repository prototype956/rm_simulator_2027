use super::*;

#[test]
fn repeated_reset_restores_robot_and_keeps_global_time_and_ids() {
    let mut world = World::new();
    world.insert_resource(TrainingRound::default());
    world.insert_resource(RoundFence::default());
    world.insert_resource(InitialScene::default());
    let config = SimulationConfig {
        combat: CombatConfig {
            controlled_allowance: AllowanceConfig::Limited { initial: 11 },
            ..default()
        },
        ..default()
    };
    world.insert_resource(config);
    let mut clock = Time::<Fixed>::default();
    clock.advance_by(Duration::from_secs(10));
    world.insert_resource(clock);
    world.insert_resource(ControllerState::default());
    world.insert_resource(ProjectileStatistics::default());
    world.insert_resource(SubscribeAutoAim(std::sync::atomic::AtomicBool::new(true)));
    let initial = Transform::from_xyz(1.0, 1.0, 2.0);
    let root = world
        .spawn((
            CombatRobotBundle::training(
                RobotId(1),
                crate::robomaster::prelude::Team::Red,
                TrainingPreset::InfantryHealthCooling,
            ),
            Controlled,
            initial,
            RigidBody::Dynamic,
        ))
        .id();
    let child = world
        .spawn((
            ChildOf(root),
            Transform::from_rotation(Quat::from_rotation_y(0.3)),
            InfantryGimbal::default(),
            RobotMember {
                root,
                id: RobotId(1),
            },
        ))
        .id();
    let launcher = world.spawn(DartLaunch).id();
    let original_light = Handle::<StandardMaterial>::default();
    world
        .entity_mut(child)
        .insert(MeshMaterial3d(original_light.clone()));
    world.get_mut::<RobotCombatState>(root).unwrap().allowance = FireAllowance::Limited(11);
    capture_initial_robot(&mut world, root);
    for expected in 2..5 {
        world.get_mut::<Transform>(root).unwrap().translation = Vec3::splat(99.0);
        world.get_mut::<LinearVelocity>(root).unwrap().0 = Vec3::splat(10.0);
        world.get_mut::<InfantryGimbal>(child).unwrap().pitch = 1.0;
        let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
        state.life.hp = 0;
        state.life.status = LifeStatus::Dead;
        state.heat.round_locked = true;
        state.shooter.actual_shots = 9;
        state.allowance = FireAllowance::Limited(0);
        state.shooter.pending = Some(shooting::PendingShot {
            request: shooting::FireRequest {
                id: 90,
                robot: RobotId(1),
                source: shooting::FireSource::Manual,
                requested_at: Duration::ZERO,
            },
            ready_at: Duration::from_secs(99),
        });
        world.entity_mut(child).insert(ExtinguishedLight {
            original: original_light.clone(),
        });
        world.entity_mut(root).insert(CombatDead);
        world.entity_mut(child).insert(CombatDead);
        let bullet = world.spawn(Projectile).id();
        record_event(&mut world, Duration::from_secs(10), "old event".into());
        request_scene_reset(&mut world);
        apply_requested_reset(&mut world);
        assert!(world.get_entity(bullet).is_err());
        assert!(world.get_entity(launcher).is_ok());
        assert!(world.get::<ExtinguishedLight>(child).is_none());
        assert_eq!(
            world
                .get::<MeshMaterial3d<StandardMaterial>>(child)
                .unwrap()
                .0,
            original_light
        );
        assert_eq!(
            world.get::<Transform>(root).unwrap().translation,
            initial.translation
        );
        assert_eq!(world.get::<LinearVelocity>(root).unwrap().0, Vec3::ZERO);
        assert_eq!(world.get::<InfantryGimbal>(child).unwrap().pitch, 0.0);
        assert!(
            world.get::<CombatDead>(root).is_none() && world.get::<CombatDead>(child).is_none()
        );
        let state = world.get::<RobotCombatState>(root).unwrap();
        assert_eq!(state.life.hp, 350);
        assert!(!state.heat.is_locked());
        assert_eq!(state.shooter.actual_shots, 0);
        assert!(state.shooter.pending.is_none());
        assert_eq!(state.allowance, FireAllowance::Limited(11));
        assert!(
            !world
                .resource::<SubscribeAutoAim>()
                .load(std::sync::atomic::Ordering::Acquire)
        );
        let round = world.resource::<TrainingRound>();
        assert_eq!(round.id, expected);
        assert!(round.events.is_empty());
        assert_eq!(*world.resource::<RoundFence>().0.lock().unwrap(), expected);
        assert_eq!(
            world.resource::<Time<Fixed>>().elapsed(),
            Duration::from_secs(10)
        );
    }
}

#[test]
fn stale_capture_cannot_publish_into_new_round() {
    let fence = RoundFence::default();
    let capture_round = 1;
    assert!(fence.lock_round(capture_round).is_some());
    // Simulates a GPU callback completing after the main world has reset.
    *fence.0.lock().unwrap() = 2;
    assert!(fence.lock_round(capture_round).is_none());
    assert!(fence.lock_round(2).is_some());
}

#[test]
fn bounded_events_keep_monotonic_ids_and_relative_simulation_time() {
    let mut world = World::new();
    world.insert_resource(TrainingRound {
        started_at: Duration::from_secs(10),
        ..default()
    });
    for _ in 0..(EVENT_CAPACITY + 3) {
        record_event(&mut world, Duration::from_secs(12), "hit".into());
    }
    let round = world.resource::<TrainingRound>();
    assert_eq!(round.events.len(), EVENT_CAPACITY);
    assert_eq!(round.dropped_events, 3);
    assert_eq!(round.events.front().unwrap().id, 4);
    assert_eq!(round.events.back().unwrap().at, Duration::from_secs(2));
}

#[test]
fn real_physics_reset_while_touching_ground_preserves_contact_islands() {
    use bevy::time::TimeUpdateStrategy;
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        ResetPlugin,
    ));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / 60.0,
    )));
    app.add_message::<AssetEvent<Mesh>>();
    app.init_resource::<Assets<Mesh>>();
    app.insert_resource(SimulationConfig::default());
    let world = app.world_mut();
    world.spawn((
        RigidBody::Static,
        Collider::cuboid(30.0, 0.1, 30.0),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));
    let root = world
        .spawn((
            CombatRobotBundle::training(
                RobotId(1),
                crate::robomaster::prelude::Team::Red,
                TrainingPreset::InfantryHealthCooling,
            ),
            Controlled,
            RigidBody::Dynamic,
            Collider::cuboid(0.3, 0.3, 0.3),
            Transform::from_xyz(0.0, 0.15, 0.0),
        ))
        .id();
    capture_initial_robot(world, root);
    app.finish();
    app.cleanup();
    for _ in 0..60 {
        app.update();
    }
    for expected in 2..12 {
        request_scene_reset(app.world_mut());
        for _ in 0..6 {
            app.update();
        }
        assert_eq!(app.world().resource::<TrainingRound>().id, expected);
        let pos = app.world().get::<Position>(root).unwrap().0;
        assert!(
            pos.is_finite() && pos.y > 0.1 && pos.y < 0.2,
            "unexpected reset pose {pos}"
        );
    }
}
