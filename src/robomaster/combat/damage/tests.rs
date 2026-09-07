use super::*;
use crate::config::SimulationConfig;
use crate::robomaster::combat::shooting::{
    FireRejection, FireRequest, FireRequestResult, FireResult, FireSource, ShootingPlugin,
    request_fire,
};
use crate::robomaster::combat::{CombatRobotBundle, TrainingPreset};
use crate::robomaster::prelude::{ArmorLabel, ArmorSpec, Projectile, Side, SmallArmorLabel, Team};
use bevy::ecs::system::RunSystemOnce;

fn fixture() -> World {
    let mut app = App::new();
    app.add_plugins((ShootingPlugin, DamagePlugin));
    let world = app.world_mut();
    world.init_resource::<Messages<CollisionStart>>();
    world.insert_resource(Time::<Fixed>::default());
    world.insert_resource(ProjectileStatistics::default());
    world.insert_resource(ControllerState::default());
    world.insert_resource(SubscribeAutoAim(std::sync::atomic::AtomicBool::new(true)));
    world.insert_resource(Assets::<StandardMaterial>::default());
    world.insert_resource(
        toml::from_str::<SimulationConfig>(include_str!("../../../../config.toml")).unwrap(),
    );
    std::mem::take(world)
}

fn robot(world: &mut World, id: u64, team: Team) -> (Entity, Entity) {
    let root = world
        .spawn((
            CombatRobotBundle::training(RobotId(id), team, TrainingPreset::InfantryHealthCooling),
            RigidBody::Dynamic,
            Collider::sphere(0.2),
            Transform::default(),
            LinearVelocity(Vec3::X),
            AngularVelocity(Vec3::Y),
        ))
        .id();
    let armor = world
        .spawn((
            Armor {
                name: "test armor".into(),
                team,
                spec: ArmorSpec::Small(SmallArmorLabel::Three),
                label: ArmorLabel::Three,
            },
            RobotMember {
                root,
                id: RobotId(id),
            },
            ChildOf(root),
            Transform::default(),
        ))
        .id();
    (root, armor)
}

fn projectile(world: &mut World, id: u64, shooter: u64) -> Entity {
    world
        .spawn((
            Projectile,
            ProjectileShot {
                id,
                request: FireRequest {
                    id,
                    robot: RobotId(shooter),
                    source: FireSource::Manual,
                    requested_at: Duration::ZERO,
                },
                fired_at: Duration::ZERO,
                muzzle_pose: Transform::default(),
                initial_velocity: Vec3::Z * 25.0,
            },
            RigidBody::Dynamic,
            Collider::sphere(0.0085),
            LinearVelocity(Vec3::Z * 25.0),
        ))
        .id()
}

fn hit(world: &mut World, projectile: Entity, collider: Entity, body: Entity) {
    resolve_impact(
        world,
        CollisionStart {
            collider1: projectile,
            collider2: collider,
            body1: Some(projectile),
            body2: Some(body),
        },
    );
}

#[test]
fn contact_batch_deduplicates_descendant_armors_and_consumes_body_hits() {
    let mut world = fixture();
    let (shooter, _) = robot(&mut world, 1, Team::Red);
    let (target, armor) = robot(&mut world, 2, Team::Blue);
    let mesh1 = world.spawn(ChildOf(armor)).id();
    let mesh2 = world.spawn(ChildOf(armor)).id();
    let bullet = projectile(&mut world, 1, 1);
    hit(&mut world, bullet, mesh1, target);
    hit(&mut world, bullet, mesh2, target);
    assert_eq!(world.get::<RobotCombatState>(target).unwrap().life.hp, 330);
    assert_eq!(
        world
            .get::<RobotCombatState>(shooter)
            .unwrap()
            .damage
            .damage_dealt,
        20
    );
    assert_eq!(world.resource::<ProjectileStatistics>().armor_hit_count, 1);
    assert!(world.get_entity(bullet).is_err());
    let bullet = projectile(&mut world, 2, 1);
    hit(&mut world, bullet, target, target);
    hit(&mut world, bullet, armor, target);
    assert_eq!(world.get::<RobotCombatState>(target).unwrap().life.hp, 330);
    assert_eq!(world.resource::<ProjectileStatistics>().armor_hit_count, 1);
}

#[test]
fn lethal_hit_clamps_damage_cancels_feed_preserves_inertia_and_round_lock() {
    let mut world = fixture();
    let (shooter, shooter_armor) = robot(&mut world, 1, Team::Red);
    let (target, armor) = robot(&mut world, 2, Team::Blue);
    world.entity_mut(target).insert(Controlled);
    // A shot from the future victim remains dangerous after its owner dies.
    let in_flight = projectile(&mut world, 99, 2);
    let queued = FireRequest {
        id: 100,
        robot: RobotId(2),
        source: FireSource::Manual,
        requested_at: Duration::ZERO,
    };
    {
        let mut state = world.get_mut::<RobotCombatState>(target).unwrap();
        state.life.hp = 7;
        state.heat.round_locked = true;
        state.shooter.pending = Some(super::super::shooting::PendingShot {
            request: queued,
            ready_at: Duration::from_secs(5),
        });
    }
    super::super::heat::add_shot_heat(&mut world, target, Duration::ZERO);
    request_fire(&mut world, RobotId(2), FireSource::Manual);
    let bullet = projectile(&mut world, 1, 1);
    hit(&mut world, bullet, armor, target);
    let state = world.get::<RobotCombatState>(target).unwrap();
    assert_eq!(state.life.hp, 0);
    assert_eq!(state.life.status, LifeStatus::Dead);
    assert_eq!(state.heat.current(), 0.0);
    assert!(state.heat.round_locked);
    assert!(state.shooter.pending.is_none());
    assert!(world.get::<CombatDead>(target).is_some());
    assert!(world.get::<CombatDead>(armor).is_some());
    assert!(world.get::<Collider>(target).is_some());
    assert_eq!(world.get::<LinearVelocity>(target).unwrap().0, Vec3::X);
    assert_eq!(world.get::<AngularVelocity>(target).unwrap().0, Vec3::Y);
    assert!(!world.resource::<SubscribeAutoAim>().load(Ordering::Acquire));
    let stats = &world.get::<RobotCombatState>(shooter).unwrap().damage;
    assert_eq!(
        (stats.damaging_hits, stats.damage_dealt, stats.kills),
        (1, 7, 1)
    );
    let cancelled: Vec<_> = world
        .resource_mut::<Messages<FireRequestResult>>()
        .drain()
        .collect();
    assert_eq!(cancelled.len(), 2);
    assert!(
        cancelled
            .iter()
            .all(|r| r.result == FireResult::Rejected(FireRejection::Dead))
    );
    request_fire(&mut world, RobotId(2), FireSource::Manual);
    super::super::shooting::execute_fire_requests(&mut world);
    let refused: Vec<_> = world
        .resource_mut::<Messages<FireRequestResult>>()
        .drain()
        .collect();
    assert_eq!(refused.len(), 1);
    assert_eq!(refused[0].result, FireResult::Rejected(FireRejection::Dead));
    let events: Vec<_> = world
        .resource_mut::<Messages<DamageApplied>>()
        .drain()
        .collect();
    assert_eq!((events[0].nominal, events[0].actual), (20, 7));
    // Hitting a corpse is an armor contact, but not another damage or kill event.
    let bullet = projectile(&mut world, 2, 1);
    hit(&mut world, bullet, armor, target);
    assert_eq!(
        world
            .resource_mut::<Messages<DamageApplied>>()
            .drain()
            .count(),
        0
    );
    assert_eq!(
        world.get::<RobotCombatState>(shooter).unwrap().damage.kills,
        1
    );
    hit(&mut world, in_flight, shooter_armor, shooter);
    assert_eq!(world.get::<RobotCombatState>(shooter).unwrap().life.hp, 330);
}

#[test]
fn self_friendly_environment_and_noncombat_targets_do_not_cause_robot_damage() {
    let mut world = fixture();
    let (shooter, self_armor) = robot(&mut world, 1, Team::Red);
    let (friend, friend_armor) = robot(&mut world, 2, Team::Red);
    let bullet = projectile(&mut world, 1, 1);
    hit(&mut world, bullet, self_armor, shooter);
    hit(&mut world, bullet, friend_armor, friend);
    assert!(world.get_entity(bullet).is_ok());
    assert_eq!(world.resource::<ProjectileStatistics>().armor_hit_count, 0);
    let wall = world.spawn_empty().id();
    hit(&mut world, bullet, wall, wall);
    assert!(world.get_entity(bullet).is_err());
    assert_eq!(world.get::<RobotCombatState>(shooter).unwrap().life.hp, 350);
    assert_eq!(world.get::<RobotCombatState>(friend).unwrap().life.hp, 350);
}

#[test]
fn dead_light_uses_its_own_material_and_keeps_the_original_for_reset() {
    let mut world = fixture();
    let (root, _) = robot(&mut world, 1, Team::Red);
    world.get_mut::<RobotCombatState>(root).unwrap().life.status = LifeStatus::Dead;
    let original = world
        .resource_mut::<Assets<StandardMaterial>>()
        .add(StandardMaterial {
            emissive: LinearRgba::WHITE,
            ..default()
        });
    let light = world
        .spawn((
            RobotMember {
                root,
                id: RobotId(1),
            },
            MeshMaterial3d(original.clone()),
            LightStrip {
                side: Side::Left,
                visibility_id: 1,
                mask_triangles: vec![],
            },
        ))
        .id();
    world.run_system_once(extinguish_lights).unwrap();
    let dark = &world
        .get::<MeshMaterial3d<StandardMaterial>>(light)
        .unwrap()
        .0;
    assert_ne!(dark, &original);
    let materials = world.resource::<Assets<StandardMaterial>>();
    assert_eq!(materials.get(dark).unwrap().emissive, LinearRgba::BLACK);
    assert_eq!(
        materials.get(&original).unwrap().emissive,
        LinearRgba::WHITE
    );
    assert_eq!(
        world.get::<ExtinguishedLight>(light).unwrap().original,
        original
    );
}

/// Real Avian integration: a 25 m/s bullet above the floor must hit thin armor,
/// not be consumed by a distant speculative floor pair or tunnel through the plate.
#[test]
fn swept_projectile_hits_thin_armor_without_speculative_floor_damage() {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default().with_collision_hooks::<ProjectileContactHooks>(),
        DamagePlugin,
    ));
    app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
        Duration::from_secs_f64(1.0 / 60.0),
    ));
    app.insert_resource(Gravity(Vec3::ZERO));
    app.add_message::<AssetEvent<Mesh>>();
    app.init_resource::<Assets<Mesh>>();
    app.init_resource::<Assets<StandardMaterial>>();
    app.init_resource::<ProjectileStatistics>();
    app.init_resource::<Messages<FireRequestResult>>();
    let world = app.world_mut();
    world.spawn((
        RigidBody::Static,
        Collider::cuboid(50.0, 0.1, 50.0),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));
    world.spawn(CombatRobotBundle::training(
        RobotId(1),
        Team::Red,
        TrainingPreset::InfantryHealthCooling,
    ));
    let target = world
        .spawn((
            CombatRobotBundle::training(
                RobotId(2),
                Team::Blue,
                TrainingPreset::InfantryHealthCooling,
            ),
            RigidBody::Static,
            Transform::default(),
        ))
        .id();
    world.spawn((
        Armor {
            name: "thin plate".into(),
            team: Team::Blue,
            spec: ArmorSpec::Small(SmallArmorLabel::Three),
            label: ArmorLabel::Three,
        },
        ChildOf(target),
        RobotMember {
            root: target,
            id: RobotId(2),
        },
        Collider::cuboid(0.2, 0.2, 0.01),
        Transform::from_xyz(0.0, 0.3, 1.0),
    ));
    let bullet = projectile(world, 1, 1);
    world.entity_mut(bullet).insert((
        Transform::from_xyz(0.0, 0.3, 0.0),
        SweptCcd::default(),
        SpeculativeMargin::ZERO,
        ActiveCollisionHooks::MODIFY_CONTACTS,
        ProjectileSweepStart(Vec3::new(0.0, 0.3, 0.0)),
    ));
    app.finish();
    app.cleanup();
    for _ in 0..12 {
        app.update();
    }
    assert!(
        app.world().get_entity(bullet).is_err(),
        "unconsumed bullet: position={:?}, velocity={:?}, elapsed={:?}",
        app.world().get::<Position>(bullet),
        app.world().get::<LinearVelocity>(bullet),
        app.world().resource::<Time<Fixed>>().elapsed()
    );
    assert_eq!(
        app.world().get::<RobotCombatState>(target).unwrap().life.hp,
        330
    );
    assert_eq!(
        app.world()
            .resource::<ProjectileStatistics>()
            .armor_hit_count,
        1
    );
}

#[test]
fn manual_and_remote_gimbals_cannot_be_driven_after_death() {
    use crate::components::InfantryGimbal;
    use crate::systems::{gimbal_controls, remote_gimbal_controls, update_auto_aim_subscription};
    for controlled in [true, false] {
        let mut world = fixture();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_millis(20));
        world.insert_resource(time);
        let _ = robot(&mut world, 1, Team::Red);
        let (root, armor) = robot(&mut world, 2, Team::Blue);
        let gimbal = world
            .spawn((
                InfantryGimbal::default(),
                Transform::default(),
                ChildOf(root),
                RobotMember {
                    root,
                    id: RobotId(2),
                },
            ))
            .id();
        for entity in [root, gimbal] {
            if controlled {
                world.entity_mut(entity).insert(Controlled);
            } else {
                world.entity_mut(entity).insert(ActiveSlapper);
            }
        }
        world
            .resource::<SubscribeAutoAim>()
            .store(false, Ordering::Release);
        world.resource_mut::<ControllerState>().controlled.gimbal = Vec2::X;
        world.resource_mut::<ControllerState>().remote.gimbal = Vec2::X;
        if controlled {
            world.run_system_once(gimbal_controls).unwrap();
        } else {
            world.run_system_once(remote_gimbal_controls).unwrap();
        }
        let before = *world.get::<Transform>(gimbal).unwrap();
        assert_ne!(before.rotation, Quat::IDENTITY);
        world.get_mut::<RobotCombatState>(root).unwrap().life.hp = 1;
        let bullet = projectile(&mut world, 1, 1);
        hit(&mut world, bullet, armor, root);
        world.resource_mut::<ControllerState>().controlled.gimbal = Vec2::X;
        world.resource_mut::<ControllerState>().remote.gimbal = Vec2::X;
        // The real drive systems must be skipped by their dead-entity query filters.
        if controlled {
            assert!(world.run_system_once(gimbal_controls).is_err());
        } else {
            assert!(world.run_system_once(remote_gimbal_controls).is_err());
        }
        assert_eq!(*world.get::<Transform>(gimbal).unwrap(), before);
        world.resource_mut::<ControllerState>().controlled.auto_aim = true;
        world.run_system_once(update_auto_aim_subscription).unwrap();
        assert!(!world.resource::<SubscribeAutoAim>().load(Ordering::Acquire));
    }
}
