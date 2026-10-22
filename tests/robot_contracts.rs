//! Real-asset robot contracts. No window, sockets, screenshots or desktop input.
use avian3d::prelude::*;
use bevy::{ecs::system::RunSystemOnce, prelude::*, time::TimeUpdateStrategy};
use daedalus::{
    components::*,
    config::SimulationConfig,
    robomaster::{
        combat::*,
        prelude::{ArmorRoot, ArmorType},
    },
    robot_motion::*,
    robots::*,
    training::environment::build_app,
};
use std::time::Duration;

fn build_scene(controlled: RobotProfile, target: RobotProfile) -> App {
    let mut config = SimulationConfig::load_from(std::path::Path::new(
        "configs/rmul_2027_multirobot_infantry.toml",
    ))
    .unwrap();
    config.robots = Some(RobotProfiles { controlled, target });
    build_app(config, std::env::current_dir().unwrap().join("assets")).unwrap()
}
fn step(app: &mut App, ms: usize) {
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(1)));
    for _ in 0..ms {
        app.update();
    }
}
fn place(app: &mut App, e: Entity, t: Transform) {
    app.world_mut().entity_mut(e).insert((
        t,
        Position(t.translation),
        Rotation(t.rotation),
        RigidBody::Kinematic,
        LinearVelocity::ZERO,
        AngularVelocity::ZERO,
    ));
}
fn scene(profile: RobotProfile) -> (App, Entity) {
    let mut app = build_scene(profile, RobotProfile::Standard);
    let w = app.world_mut();
    let roots: Vec<_> = w
        .query::<(Entity, &RobotIdentity)>()
        .iter(w)
        .map(|(e, id)| (e, id.id))
        .collect();
    let own = roots
        .iter()
        .find(|(_, id)| *id == CONTROLLED_ROBOT_ID)
        .unwrap()
        .0;
    for (e, _) in roots {
        let t = *app.world().get::<Transform>(e).unwrap();
        place(&mut app, e, t);
    }
    (app, own)
}

#[test]
fn profile_geometry_and_tunnel_clearance() {
    for profile in [
        RobotProfile::Standard,
        RobotProfile::WheelLeg,
        RobotProfile::Low,
        RobotProfile::Folding,
        RobotProfile::HeavyTarget,
    ] {
        let mut app = build_scene(RobotProfile::Standard, profile);
        let w = app.world_mut();
        let root = w
            .query::<(Entity, &RobotIdentity)>()
            .iter(w)
            .find(|(_, id)| id.id == TARGET_ROBOT_ID)
            .unwrap()
            .0;
        assert_eq!(
            w.query::<(&ArmorRoot, &RobotMember)>()
                .iter(w)
                .filter(|(_, m)| m.root == root)
                .count(),
            4
        );
        let report = geometry_report(w, root);
        if profile == RobotProfile::Low {
            assert!(
                report["top_above_floor_m"].as_f64().unwrap() <= 0.230,
                "{report}"
            );
        }
        if profile == RobotProfile::Folding {
            let mut rig = w.get::<RobotRig>(root).unwrap().clone();
            set_mounts(w, profile, &rig, 1., 0.);
            rig.fold = 1.;
            rig.requested_fold = true;
            w.entity_mut(root).insert(rig.clone());
            let report = geometry_report(w, root);
            assert!(
                report["top_above_floor_m"].as_f64().unwrap() <= 0.230,
                "{report}"
            );
        }
        // The 350 mm bridge is above an 800 mm wide passage, not an 800 mm roof.
        let tunnel = Transform::from_xyz(-2.5, profile.root_height(), -3.46);
        let admitted = check_pose(w, root, tunnel).is_ok();
        assert_eq!(admitted, profile.compact(), "tunnel profile {profile:?}");
        // No small armor changes size when the chassis is compressed.
        for (armor, m) in w
            .query::<(&ArmorRoot, &RobotMember)>()
            .iter(w)
            .filter(|(_, m)| m.root == root)
        {
            assert_eq!(
                armor.spec.armor_type(),
                if profile == RobotProfile::HeavyTarget {
                    ArmorType::Large
                } else {
                    ArmorType::Small
                }
            );
            assert_eq!(m.id, TARGET_ROBOT_ID);
        }
    }
}

#[test]
fn camera_mounts_and_profile_reset() {
    let mut app = build_scene(RobotProfile::Folding, RobotProfile::WheelLeg);
    let w = app.world_mut();
    let own = w
        .query_filtered::<Entity, (With<Controlled>, With<RobotIdentity>)>()
        .single(w)
        .unwrap();
    let camera = w
        .spawn((
            MainCamera {
                follow_offset: Vec3::new(0., 3., 2.),
            },
            Transform::default(),
        ))
        .id();
    w.insert_resource(CameraMode(FollowingType::Robot));
    let view = w
        .query_filtered::<Entity, (With<Controlled>, With<InfantryViewOffset>)>()
        .single(w)
        .unwrap();
    let muzzle = w
        .query_filtered::<Entity, (With<Controlled>, With<InfantryLaunchOffset>)>()
        .single(w)
        .unwrap();
    for fold in [0., 0.5, 1.] {
        let rig = w.get::<RobotRig>(own).unwrap().clone();
        set_mounts(w, RobotProfile::Folding, &rig, fold, 0.);
        w.run_system_once(daedalus::systems::update_camera_follow)
            .unwrap();
        let camera = w.get::<Transform>(camera).unwrap();
        let v = physical_matrix(w, view).transform_point3(Vec3::ZERO);
        let (_, q, _) = physical_matrix(w, muzzle).to_scale_rotation_translation();
        assert!(camera.translation.distance(v) < 1e-5);
        assert!((camera.rotation * Vec3::NEG_Z).dot(q * Vec3::Y) > 0.99999);
    }
    assert_eq!(
        w.query_filtered::<Entity, (With<ActiveSlapper>, With<RobotIdentity>)>()
            .iter(w)
            .count(),
        1
    );
    w.resource_mut::<SimulationConfig>().robots = Some(RobotProfiles {
        controlled: RobotProfile::Low,
        target: RobotProfile::HeavyTarget,
    });
    reset::request_scene_reset(w);
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(1)));
    let mut ready = false;
    for _ in 0..2000 {
        app.update();
        let w = app.world_mut();
        let profiles: Vec<_> = w.query::<&RobotProfile>().iter(w).copied().collect();
        if profiles.contains(&RobotProfile::Low) && profiles.contains(&RobotProfile::HeavyTarget) {
            if w.query::<&ArmorRoot>().iter(w).count() == 8 {
                ready = true;
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(ready, "profile reset did not rebuild both instances");
    let w = app.world_mut();
    let roots: Vec<_> = w
        .query::<(&RobotIdentity, &RobotCombatState, &RobotProfile)>()
        .iter(w)
        .collect();
    assert_eq!(roots.len(), 2);
    assert_eq!(
        roots
            .iter()
            .find(|(_, _, p)| **p == RobotProfile::HeavyTarget)
            .unwrap()
            .1
            .life
            .hp,
        450
    );
}

#[test]
fn urdf_links_mounts_and_contact() {
    let mut app = build_scene(RobotProfile::WheelLeg, RobotProfile::WheelLeg);
    let w = app.world_mut();
    let roots: Vec<_> = w
        .query_filtered::<Entity, With<RobotIdentity>>()
        .iter(w)
        .collect();
    for root in &roots {
        w.entity_mut(*root).insert((
            RigidBody::Kinematic,
            LinearVelocity::ZERO,
            AngularVelocity::ZERO,
        ));
        let rig = w.get::<RobotRig>(*root).unwrap().clone();
        let native = rig.urdf.as_ref().unwrap();
        assert_eq!(native.chains.len(), 2);
        let hulls = w
            .query::<(&Name, &RobotMember, &Collider)>()
            .iter(w)
            .filter(|(n, m, _)| m.root == *root && n.starts_with("UrdfHull_"))
            .count();
        assert!(hulls > 200, "base CAD parts must have separate hulls");
        assert!(
            w.get::<Collider>(*root).is_none(),
            "no fixed ground support cylinder"
        );
        let joints: Vec<_> = native
            .chains
            .iter()
            .flatten()
            .map(|e| (*e, *w.get::<Transform>(*e).unwrap()))
            .collect();
        let markers: Vec<_> = w
            .query::<(
                Entity,
                &RobotMember,
                Option<&ArmorRoot>,
                Option<&InfantryLaunchOffset>,
                Option<&InfantryViewOffset>,
            )>()
            .iter(w)
            .filter(|(_, m, a, b, c)| {
                m.root == *root && (a.is_some() || b.is_some() || c.is_some())
            })
            .map(|(e, _, _, _, _)| (e, physical_matrix(w, e).transform_point3(Vec3::ZERO)))
            .collect();
        assert_eq!(
            markers.len(),
            if w.get::<Controlled>(*root).is_some() {
                6
            } else {
                5
            }
        );
        let before = geometry_report(w, *root);
        set_mounts(w, RobotProfile::WheelLeg, &rig, 0., 0.2);
        for (e, p) in &markers {
            let moved = physical_matrix(w, *e).transform_point3(Vec3::ZERO);
            assert!(moved.distance(*p + Vec3::Y * 0.2) < 0.0001);
        }
        let after = geometry_report(w, *root);
        assert!(
            (before["visible_min_m"][1].as_f64().unwrap()
                - after["visible_min_m"][1].as_f64().unwrap())
            .abs()
                < 0.0001,
            "wheel contact moved: {before} {after}"
        );
        let pose = *w.get::<Transform>(*root).unwrap();
        check_pose(w, *root, pose).unwrap();
        set_mounts(w, RobotProfile::WheelLeg, &rig, 0., 0.);
        for (e, t) in joints {
            let a = w.get::<Transform>(e).unwrap();
            assert!(a.translation.distance(t.translation) < 0.0001);
            assert!(a.rotation.angle_between(t.rotation) < 0.001);
        }
    }
    reset::request_scene_reset(w);
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(1)));
    for _ in 0..10 {
        app.update();
    }
    for root in roots {
        let rig = app.world().get::<RobotRig>(root).unwrap();
        assert!(rig.urdf.is_some());
        assert_eq!(rig.lift_m, 0.);
    }
}

#[test]
fn folding_interlocks_and_feed() {
    let (mut app, own) = scene(RobotProfile::Folding);
    let home = *app.world().get::<Transform>(own).unwrap();
    app.world_mut()
        .get_mut::<RobotCombatState>(own)
        .unwrap()
        .shooter
        .mechanics
        .launch_delay = Duration::from_millis(200);
    shooting::request_fire(
        app.world_mut(),
        CONTROLLED_ROBOT_ID,
        shooting::FireSource::Manual,
    );
    step(&mut app, 1);
    assert!(
        app.world()
            .get::<RobotCombatState>(own)
            .unwrap()
            .shooter
            .pending
            .is_some()
    );
    request_fold(app.world_mut(), own, true).unwrap();
    step(&mut app, 400);
    let r = app.world().get::<RobotRig>(own).unwrap();
    assert!(r.fold > 0.45 && r.fold < 0.55);
    assert!(!r.fire_ready());
    let state = app.world().get::<RobotCombatState>(own).unwrap();
    assert_eq!(state.allowance, FireAllowance::Limited(200));
    assert_eq!(state.shooter.actual_shots, 0);
    assert_eq!(state.heat.current(), 0.);
    step(&mut app, 450);
    assert!(app.world().get::<RobotRig>(own).unwrap().fold > 0.999);
    let tunnel = Transform::from_xyz(-2.5, RobotProfile::Folding.root_height(), -3.46);
    place(&mut app, own, tunnel);
    assert!(request_fold(app.world_mut(), own, false).is_err());
    assert!(app.world().get::<RobotRig>(own).unwrap().requested_fold);
    place(&mut app, own, home);
    request_fold(app.world_mut(), own, false).unwrap();
    step(&mut app, 950);
    assert!(app.world().get::<RobotRig>(own).unwrap().fire_ready());
    shooting::request_fire(
        app.world_mut(),
        CONTROLLED_ROBOT_ID,
        shooting::FireSource::Manual,
    );
    step(&mut app, 201);
    assert_eq!(
        app.world()
            .get::<RobotCombatState>(own)
            .unwrap()
            .shooter
            .actual_shots,
        1
    );
    assert_eq!(
        app.world().get::<RobotCombatState>(own).unwrap().allowance,
        FireAllowance::Limited(199)
    );
}

#[test]
fn scripted_traversal_paths() {
    for (profile, kind, initial, ms) in [
        (
            RobotProfile::WheelLeg,
            MotionKind::Climb,
            Vec3::new(-2.5, RobotProfile::WheelLeg.root_height(), 1.7),
            3000,
        ),
        (
            RobotProfile::Low,
            MotionKind::Tunnel,
            Vec3::new(-3.1, RobotProfile::Low.root_height(), -3.46),
            14000,
        ),
        (
            RobotProfile::Folding,
            MotionKind::Tunnel,
            Vec3::new(-3.1, RobotProfile::Folding.root_height(), -3.46),
            14000,
        ),
    ] {
        let (mut app, root) = scene(profile);
        let pose = Transform::from_translation(initial);
        place(&mut app, root, pose);
        let p = RobotProgram::new(pose, MotionPlan { kind, ..default() });
        let _samples = validate_programs(app.world_mut(), &[(root, p.clone())]).unwrap();
        app.world_mut().entity_mut(root).insert(p);
        for _ in 0..ms / 100 {
            step(&mut app, 100);
            let p = app.world().get::<RobotProgram>(root).unwrap();
            assert!(p.failed.is_none(), "{:?}: {:?}", kind, p.failed);
        }
    }
}

#[test]
fn wheel_leg_lift_and_reset() {
    let (mut app, root) = scene(RobotProfile::WheelLeg);
    app.world_mut()
        .get_mut::<RobotRig>(root)
        .unwrap()
        .requested_lift_m = 0.2;
    step(&mut app, 250);
    assert!((app.world().get::<RobotRig>(root).unwrap().lift_m - 0.2).abs() < 1e-5);
    assert!(
        app.world().get::<RobotRig>(root).unwrap().fire_ready(),
        "wheel-leg lift must not inherit folding interlock"
    );
    reset::request_scene_reset(app.world_mut());
    step(&mut app, 5);
    assert_eq!(app.world().get::<RobotRig>(root).unwrap().lift_m, 0.);
}
