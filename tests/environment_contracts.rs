//! Map/referee physics and public training API regressions. Run serially.

mod map_rules {
    use avian3d::prelude::*;
    use bevy::prelude::*;
    use bevy::render::{RenderPlugin, settings::WgpuSettings};
    use bevy::time::TimeUpdateStrategy;
    use daedalus::{
        components::*,
        config::SimulationConfig,
        gimbal_actuator::*,
        robomaster::{combat::*, prelude::*},
        setup,
        statistic::ProjectileStatistics,
        systems::*,
    };
    use std::sync::atomic::AtomicBool;
    use std::time::{Duration, Instant};
    fn advance(app: &mut App, ms: usize) {
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(1)));
        for _ in 0..ms {
            app.update();
        }
    }
    fn robot(app: &mut App, id: RobotId) -> Entity {
        let w = app.world_mut();
        w.query::<(Entity, &RobotIdentity)>()
            .iter(w)
            .find(|(_, i)| i.id == id)
            .unwrap()
            .0
    }
    fn teleport(app: &mut App, e: Entity, p: Vec3) {
        let w = app.world_mut();
        let mut t = *w.get::<Transform>(e).unwrap();
        t.translation = p;
        w.entity_mut(e)
            .insert((t, Position(p), LinearVelocity::ZERO, AngularVelocity::ZERO));
    }
    fn reset(app: &mut App) {
        reset::request_scene_reset(app.world_mut());
        advance(app, 20);
    }
    #[test]
    fn geometry_combat_and_reset() {
        let config =
            SimulationConfig::load_from(std::path::Path::new("configs/rmul_2027_infantry.toml"))
                .unwrap();
        let mut app = App::new();
        app.add_plugins(
            DefaultPlugins
                .set(AssetPlugin {
                    file_path: std::env::current_dir()
                        .unwrap()
                        .join("assets")
                        .to_string_lossy()
                        .into(),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: None,
                    ..default()
                })
                .set(RenderPlugin {
                    render_creation: WgpuSettings {
                        backends: None,
                        ..default()
                    }
                    .into(),
                    ..default()
                })
                .disable::<bevy::winit::WinitPlugin>()
                .disable::<bevy::audio::AudioPlugin>()
                .disable::<bevy::gilrs::GilrsPlugin>()
                .disable::<bevy::log::LogPlugin>(),
        );
        app.add_plugins((
            PhysicsPlugins::default().with_collision_hooks::<damage::ProjectileContactHooks>(),
            ArmorPlugins,
            CombatPlugin,
        ));
        app.insert_resource(config)
            .insert_resource(Time::<Fixed>::from_duration(Duration::from_millis(1)))
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::ZERO))
            .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
            .insert_resource(SubstepCount(2))
            .insert_resource(SubscribeAutoAim(AtomicBool::new(false)))
            .insert_resource(GimbalCommandInbox::standalone())
            .init_resource::<ActuatorClock>()
            .init_resource::<GimbalActuator>()
            .init_resource::<GimbalActuatorTelemetry>()
            .init_resource::<ProjectileStatistics>()
            .init_resource::<ControllerState>()
            .init_resource::<ledger::CombatLedger>()
            .add_systems(Startup, (setup::setup, setup_projectile))
            .add_observer(setup::setup_vehicle)
            .add_observer(setup::setup_collision);
        app.finish();
        app.cleanup();
        let deadline = Instant::now() + Duration::from_secs(60);
        loop {
            app.update();
            let w = app.world_mut();
            let muzzles = w.query::<&InfantryLaunchOffset>().iter(w).count();
            let pending = w
                .query_filtered::<Entity, Or<(
                    With<ColliderConstructor>,
                    With<ColliderConstructorHierarchy>,
                    With<PreciousCollision>,
                )>>()
                .iter(w)
                .count();
            if muzzles == 2 && pending == 0 {
                break;
            }
            assert!(Instant::now() < deadline, "asset initialization timeout");
            std::thread::sleep(Duration::from_millis(2));
        }
        advance(&mut app, 500);
        let own = robot(&mut app, CONTROLLED_ROBOT_ID);
        let target = robot(&mut app, TARGET_ROBOT_ID);
        assert_eq!(
            app.world().get::<RobotCombatState>(own).unwrap().life.hp,
            350
        );
        // Independent handbook coordinates, not values read back from field constants.
        use bevy::ecs::system::RunSystemOnce;
        app.world_mut()
            .run_system_once(|spatial: SpatialQuery| {
                let down = |x: f32, z: f32| {
                    let origin = Vec3::new(x, 2., -z);
                    2. - spatial
                        .cast_ray(
                            origin,
                            Dir3::NEG_Y,
                            3.,
                            true,
                            &SpatialQueryFilter::default(),
                        )
                        .unwrap()
                        .distance
                };
                for sign in [1., -1.] {
                    for (x, z, height) in [
                        (-2.9, -0.99, 0.2),
                        (-2.9, 3.05, 0.2), // full 4060 mm flat deck
                        (-2.9, -1.01, 0.),
                        (-3.49, -0.5, 0.2),
                        (-1.51, -0.5, 0.2),
                        (-3.51, -0.5, 0.),
                        (-1.49, -0.5, 0.), // exact 2000 mm width
                        (-1.66, 0.8, 0.2),
                        (-1.64, 0.8, 0.4), // 150 mm curb thickness
                        (-1.575, -0.09, 0.4),
                        (-1.575, 1.69, 0.4), // 900 offset + 1800 curb
                        (-2.9, 3.07, 0.),
                        (-2.9, 3.85, 0.),    // open 800 mm slot
                        (-2.5, 3.46, 0.27),  // only the central bridge covers the slot
                        (-2.5, 2.99, 0.235), // short metal transition, not a main approach ramp
                        (-2.5, 3.95, 0.27),  // wall-side support ledge
                        (4., 0., 0.),
                    ] {
                        let actual = down(x * sign, z * sign);
                        assert!(
                            (actual - height).abs() < 0.003,
                            "height at {x},{z} side {sign}: {actual} != {height}"
                        );
                    }
                    // Metal edges at +/-175 mm from centre; sides of slot are genuinely uncovered.
                    for (x, covered) in
                        [(-2.67, true), (-2.33, true), (-2.68, false), (-2.32, false)]
                    {
                        let up = spatial.cast_ray(
                            Vec3::new(x * sign, 0.1, -3.46 * sign),
                            Dir3::Y,
                            0.5,
                            true,
                            &SpatialQueryFilter::default(),
                        );
                        assert_eq!(up.is_some(), covered, "bridge width at {x} side {sign}");
                        if let Some(hit) = up {
                            assert!((hit.distance - 0.15).abs() < 0.003);
                        }
                    }
                    assert!(
                        spatial
                            .cast_ray(
                                Vec3::new(-3.8 * sign, 0.15, -3.46 * sign),
                                if sign > 0. { Dir3::X } else { Dir3::NEG_X },
                                2.6,
                                true,
                                &SpatialQueryFilter::default()
                            )
                            .is_none(),
                        "tunnel must pass rays below bridge"
                    );
                    assert!(
                        spatial
                            .cast_ray(
                                Vec3::new(-3.8 * sign, 0.26, -3.46 * sign),
                                if sign > 0. { Dir3::X } else { Dir3::NEG_X },
                                2.6,
                                true,
                                &SpatialQueryFilter::default()
                            )
                            .is_some(),
                        "metal must block rays through its thickness"
                    );
                    assert!(
                        spatial
                            .cast_ray(
                                Vec3::new(-2.9 * sign, 0.1, 1.2 * sign),
                                if sign > 0. { Dir3::NEG_Z } else { Dir3::Z },
                                0.4,
                                true,
                                &SpatialQueryFilter::default()
                            )
                            .is_some(),
                        "platform front must be vertical"
                    );
                }
                println!(
                    "PASS handbook deck/curb/open-slot/350mm bridge/250mm clearance, both teams"
                );
            })
            .unwrap();
        // Shoot at the closest target plate using actual cached GLB geometry.
        fn fire_at_plate(app: &mut App, own: Entity, target: Entity, speed: f32) {
            let w = app.world_mut();
            let p = w
                .resource::<league::ImpactSamples>()
                .plates
                .iter()
                .filter(|(e, _)| w.get::<RobotMember>(**e).is_some_and(|m| m.root == target))
                .max_by(|(_, a), (_, b)| a.center.z.total_cmp(&b.center.z))
                .unwrap()
                .1
                .to_owned();
            let muzzle = w
                .query_filtered::<Entity, (With<InfantryLaunchOffset>, With<Controlled>)>()
                .iter(w)
                .next()
                .unwrap();
            let parent = w.get::<ChildOf>(muzzle).unwrap().parent();
            let desired = Transform::from_translation(p.center + p.normal * 0.7)
                .with_rotation(Quat::from_rotation_arc(Vec3::Y, -p.normal));
            let parent_matrix = w.get::<GlobalTransform>(parent).unwrap().to_matrix();
            w.entity_mut(muzzle).insert(Transform::from_matrix(
                parent_matrix.inverse() * desired.to_matrix(),
            ));
            w.get_mut::<RobotCombatState>(own)
                .unwrap()
                .shooter
                .mechanics
                .speed_mps = speed;
            advance(app, 1);
            shooting::request_fire(
                app.world_mut(),
                CONTROLLED_ROBOT_ID,
                shooting::FireSource::Manual,
            );
            advance(app, 150);
        }
        fire_at_plate(&mut app, own, target, 25.);
        let state = app.world().get::<RobotCombatState>(target).unwrap();
        assert_eq!(state.life.hp, 330, "actual GLB hit must deduct 20");
        assert_eq!(
            app.world().get::<RobotCombatState>(own).unwrap().allowance,
            FireAllowance::Limited(199)
        );
        println!("PASS actual projectile damage=20, allowance=199");
        fire_at_plate(&mut app, own, target, 10.);
        assert_eq!(
            app.world().get::<RobotCombatState>(target).unwrap().life.hp,
            330,
            "slow contact must not damage"
        );
        println!("PASS slow projectile rejected");
        teleport(&mut app, target, Vec3::new(-2.5, 0.44, -0.8));
        advance(&mut app, 50);
        assert!(
            app.world()
                .get::<RobotCombatState>(target)
                .unwrap()
                .league
                .defense_active(app.world().resource::<Time<Fixed>>().elapsed())
        );
        fire_at_plate(&mut app, own, target, 25.);
        assert_eq!(
            app.world().get::<RobotCombatState>(target).unwrap().life.hp,
            315,
            "highland hit must deduct 15"
        );
        println!("PASS physical highland activation and damage=15");
        teleport(&mut app, own, Vec3::new(-4.5, 0.24, -3.));
        advance(&mut app, 20);
        teleport(&mut app, own, Vec3::new(-1.9, 0.24, -3.46));
        advance(&mut app, 1);
        let now = app.world().resource::<Time<Fixed>>().elapsed();
        assert_eq!(
            app.world()
                .get::<RobotCombatState>(own)
                .unwrap()
                .effective_cooling(now),
            36
        );
        let before = app
            .world()
            .get::<RobotCombatState>(own)
            .unwrap()
            .heat
            .current();
        advance(&mut app, 100);
        let after = app
            .world()
            .get::<RobotCombatState>(own)
            .unwrap()
            .heat
            .current();
        assert!(
            (before - after - 3.6).abs() < 0.01,
            "effective cooling must actually deduct 3.6 per 100ms: {before}->{after}"
        );
        println!("PASS physical tunnel activation and actual cooling=36/s");
        reset(&mut app);
        teleport(&mut app, own, Vec3::new(-3.9, 0.24, -3.46));
        advance(&mut app, 50);
        // Sustain a planar drive velocity, leaving the solver to resolve roof/wall contacts.
        for _ in 0..80 {
            app.world_mut().get_mut::<LinearVelocity>(own).unwrap().0 = 2. * Vec3::X;
            advance(&mut app, 20);
        }
        app.world_mut().get_mut::<LinearVelocity>(own).unwrap().0 = Vec3::ZERO;
        let end = app.world().get::<Position>(own).unwrap().0;
        assert!(
            end.x > -1.5 && (end.y - 0.24).abs() < 0.04,
            "tunnel traversal blocked or lifted: {end:?}"
        );
        let now = app.world().resource::<Time<Fixed>>().elapsed();
        assert!(
            app.world()
                .get::<RobotCombatState>(own)
                .unwrap()
                .league
                .cooling_active(now)
        );
        println!("PASS actual tunnel collision clearance and continuous traversal: {end:?}");
        reset(&mut app);
        let state = app.world().get::<RobotCombatState>(own).unwrap();
        assert_eq!(state.allowance, FireAllowance::Limited(200));
        assert_eq!(state.heat.current(), 0.);
        assert_eq!(state.life.hp, 350);
        assert!(
            !state
                .league
                .cooling_active(app.world().resource::<Time<Fixed>>().elapsed())
        );
        assert!(
            app.world()
                .resource::<league::ImpactSamples>()
                .last_detected
                .is_empty()
        );
        // A body placed on the bridge remains supported. A body over the uncovered slot falls.
        teleport(&mut app, own, Vec3::new(-2.5, 0.51, -3.46));
        advance(&mut app, 500);
        let supported = app.world().get::<Position>(own).unwrap().0;
        assert!(
            (supported.y - 0.5063).abs() < 0.02,
            "bridge failed to support body: {supported:?}"
        );
        teleport(&mut app, own, Vec3::new(-3.1, 0.51, -3.46));
        advance(&mut app, 500);
        let fallen = app.world().get::<Position>(own).unwrap().0;
        assert!(
            fallen.y < 0.27,
            "uncovered gap has an invisible roof: {fallen:?}"
        );
        reset(&mut app);
        println!("PASS bridge bearing and falling through uncovered gap");
        println!("PASS full round reset");
        for _ in 0..30 {
            shooting::request_fire(
                app.world_mut(),
                CONTROLLED_ROBOT_ID,
                shooting::FireSource::Manual,
            );
            advance(&mut app, 50);
        }
        assert!(
            app.world()
                .get::<RobotCombatState>(own)
                .unwrap()
                .heat
                .cooling_locked
        );
        assert!(
            app.world()
                .get::<RobotCombatState>(own)
                .unwrap()
                .shooter
                .actual_shots
                < 30
        );
        advance(&mut app, 5000);
        assert!(
            !app.world()
                .get::<RobotCombatState>(own)
                .unwrap()
                .heat
                .is_locked()
        );
        assert_eq!(
            app.world()
                .get::<RobotCombatState>(own)
                .unwrap()
                .heat
                .current(),
            0.
        );
        println!("PASS overheat interlock and cooling-to-zero recovery");
        reset(&mut app);
        app.world_mut()
            .get_mut::<RobotCombatState>(own)
            .unwrap()
            .allowance = FireAllowance::Limited(1);
        shooting::request_fire(
            app.world_mut(),
            CONTROLLED_ROBOT_ID,
            shooting::FireSource::Manual,
        );
        advance(&mut app, 60);
        shooting::request_fire(
            app.world_mut(),
            CONTROLLED_ROBOT_ID,
            shooting::FireSource::Manual,
        );
        advance(&mut app, 20);
        let state = app.world().get::<RobotCombatState>(own).unwrap();
        assert_eq!(state.shooter.actual_shots, 1);
        assert_eq!(state.allowance, FireAllowance::Limited(0));
        println!("PASS no-ammo rejection preserves shot count");
        app.world_mut()
            .resource_mut::<SimulationConfig>()
            .combat
            .controlled = TrainingPreset::Sentry;
        app.world_mut()
            .resource_mut::<SimulationConfig>()
            .combat
            .controlled_allowance = AllowanceConfig::Limited { initial: 750 };
        reset(&mut app);
        let state = app.world().get::<RobotCombatState>(own).unwrap();
        assert_eq!(
            (
                state.life.hp,
                state.rules.heat_limit,
                state.rules.cooling_per_second
            ),
            (400, 260, 30)
        );
        assert_eq!(state.allowance, FireAllowance::Limited(750));
        assert_eq!(
            app.world()
                .get::<Infantry>(own)
                .unwrap()
                .config
                .armor
                .label(),
            ArmorLabel::Sentry
        );
        println!("PASS sentry role/resources/visual identity reset");
        for (preset, hp, heat, cooling, role, label) in [
            (
                TrainingPreset::InfantryHealthCooling,
                350,
                88,
                24,
                CombatRole::Infantry,
                ArmorLabel::Three,
            ),
            (
                TrainingPreset::InfantryHealthBurst,
                350,
                230,
                14,
                CombatRole::Infantry,
                ArmorLabel::Three,
            ),
            (
                TrainingPreset::InfantryPowerCooling,
                300,
                88,
                24,
                CombatRole::Infantry,
                ArmorLabel::Three,
            ),
            (
                TrainingPreset::InfantryPowerBurst,
                300,
                230,
                14,
                CombatRole::Infantry,
                ArmorLabel::Three,
            ),
            (
                TrainingPreset::Sentry,
                400,
                260,
                30,
                CombatRole::Sentry,
                ArmorLabel::Sentry,
            ),
        ] {
            {
                let mut config = app.world_mut().resource_mut::<SimulationConfig>();
                config.combat.controlled = preset;
                config.combat.target = preset;
            }
            reset(&mut app);
            for root in [own, target] {
                let state = app.world().get::<RobotCombatState>(root).unwrap();
                assert_eq!(
                    (
                        state.life.hp,
                        state.rules.heat_limit,
                        state.rules.cooling_per_second
                    ),
                    (hp, heat, cooling)
                );
                assert_eq!(app.world().get::<RobotIdentity>(root).unwrap().role, role);
                assert_eq!(
                    app.world()
                        .get::<Infantry>(root)
                        .unwrap()
                        .config
                        .armor
                        .label(),
                    label
                );
            }
        }
        println!("PASS all five presets for both teams after reset");
        println!("RMUL_PHYSICAL_ACCEPTANCE_OK");
    }
}

mod training_api {
    use daedalus::{
        config::SimulationConfig,
        training::{
            environment::PhysicalEnvironment,
            protocol::{Command, Environment},
        },
    };
    use serde_json::{Value, json};
    fn selection(profile: &str, preset: Option<&str>) -> Value {
        let mut v = json!({"profile":profile,"motion":{"kind":"static"}});
        if let Some(p) = preset {
            v["preset"] = json!(p);
        }
        v
    }
    #[test]
    fn profiles_resources_reproducibility_and_legacy() {
        let config = SimulationConfig::load_from(std::path::Path::new(
            "configs/rmul_2027_multirobot_infantry.toml",
        ))
        .unwrap();
        let assets = std::env::current_dir().unwrap().join("assets");
        let mut env = PhysicalEnvironment::new(config, assets.clone());
        let profiles = ["standard", "wheel_leg", "low", "folding"];
        let mut combinations = 0;
        for own_preset in ["infantry_health_cooling", "sentry"] {
            for own_profile in profiles {
                for enemy in 0..9 {
                    let target = if enemy == 8 {
                        selection("heavy_target", None)
                    } else {
                        selection(
                            profiles[enemy % 4],
                            Some(if enemy < 4 {
                                "infantry_health_cooling"
                            } else {
                                "sentry"
                            }),
                        )
                    };
                    let s = json!({"duel":{"controlled":selection(own_profile,Some(own_preset)),"target":target},"measurements":{"noise_std_px":0.,"latency_ms":0}});
                    let data = env.reset(combinations + 1, 17, &s).unwrap_or_else(|e| {
                        panic!("own={own_profile}/{own_preset} enemy={enemy}: {e}")
                    });
                    let robots = data["evaluation"]["robots"].as_array().unwrap();
                    let own = &robots[0];
                    let target = &robots[1];
                    assert_eq!(
                        own["resources"]["allowance"]["remaining"],
                        if own_preset == "sentry" { 750 } else { 200 }
                    );
                    assert_eq!(own["hp"], if own_preset == "sentry" { 400 } else { 350 });
                    assert_eq!(
                        target["hp"],
                        if enemy == 8 {
                            450
                        } else if enemy >= 4 {
                            400
                        } else {
                            350
                        }
                    );
                    assert_eq!(own["resources"]["unlimited_heat"], false);
                    assert_eq!(target["actual_shots"], 0);
                    assert_eq!(
                        data["evaluation"]["scenario"]["initial_visibility"]["plates"]
                            .as_array()
                            .unwrap()
                            .len(),
                        4
                    );
                    let data = env
                        .advance(Command {
                            valid: false,
                            yaw_rad: 0.,
                            pitch_rad: 0.,
                            distance_m: 0.,
                            fire: false,
                        })
                        .unwrap();
                    assert_eq!(data["evaluation"]["robots"][1]["actual_shots"], 0);
                    combinations += 1;
                }
            }
        }
        for (i, (preset, hp, limit, cool)) in [
            ("infantry_health_cooling", 350, 88, 24),
            ("infantry_health_burst", 350, 230, 14),
            ("infantry_power_cooling", 300, 88, 24),
            ("infantry_power_burst", 300, 230, 14),
        ]
        .iter()
        .enumerate()
        {
            let data=env.reset(100+i as u64,17,&json!({"duel":{"controlled":selection("low",Some(preset)),"target":selection("standard",Some(preset))}})).unwrap();
            let own = &data["evaluation"]["robots"][0];
            assert_eq!(own["hp"], *hp);
            assert_eq!(own["resources"]["heat_limit"], *limit);
            assert_eq!(own["resources"]["cooling_per_second"], *cool);
        }
        for (i, (side, profile, kind)) in [
            ("controlled", "wheel_leg", "climb"),
            ("target", "wheel_leg", "climb"),
            ("controlled", "wheel_leg", "jump"),
            ("target", "wheel_leg", "jump"),
            ("controlled", "folding", "tunnel"),
            ("target", "folding", "tunnel"),
        ]
        .iter()
        .enumerate()
        {
            let mut duel = json!({"controlled":selection("standard",Some("infantry_health_cooling")),"target":selection("standard",Some("infantry_health_cooling"))});
            duel[*side]["profile"] = json!(profile);
            duel[*side]["motion"] = json!({"kind":kind});
            env.reset(150 + i as u64, 17, &json!({"duel":duel}))
                .unwrap_or_else(|e| panic!("template {side} {kind}: {e}"));
            for _ in 0..50 {
                env.advance(Command {
                    valid: false,
                    yaw_rad: 0.,
                    pitch_rad: 0.,
                    distance_m: 0.,
                    fire: false,
                })
                .unwrap();
            }
        }
        // Joint moving camera + target geometry. Follow deterministic scripted states through
        // the ordinary Advance API; actions below do not contain target truth or force hits.
        let s = json!({"duel":{"controlled":{"profile":"wheel_leg","motion":{"kind":"lift"}},
        "target":{"profile":"standard","motion":{"kind":"combined","amplitude_m":0.2,"speed_m_s":0.3,"angular_speed_rad_s":1.}}},
        "measurements":{"noise_std_px":0.,"latency_ms":0}});
        let first = env.reset(200, 17, &s).unwrap();
        let mut trace = vec![];
        for _ in 0..20 {
            trace.push(
                env.advance(Command {
                    valid: false,
                    yaw_rad: 0.,
                    pitch_rad: 0.,
                    distance_m: 0.,
                    fire: false,
                })
                .unwrap()["evaluation"]["robots"]
                    .clone(),
            );
        }
        let second = env.reset(201, 17, &s).unwrap();
        assert_eq!(
            first["evaluation"]["scenario"],
            second["evaluation"]["scenario"]
        );
        for expected in trace {
            let actual = env
                .advance(Command {
                    valid: false,
                    yaw_rad: 0.,
                    pitch_rad: 0.,
                    distance_m: 0.,
                    fire: false,
                })
                .unwrap();
            assert_eq!(actual["evaluation"]["robots"], expected);
        }
        assert!(
            env.reset(202, 17, &json!({"duel":{},"unlimited_heat":true}))
                .is_err()
        );
        assert!(
            env.reset(
                203,
                17,
                &json!({"duel":{"controlled":{"profile":"heavy_target"}}})
            )
            .is_err()
        );
        let legacy = SimulationConfig::load_from(std::path::Path::new("config.toml")).unwrap();
        let mut old = PhysicalEnvironment::new(legacy, assets);
        old.reset(1, 17, &json!({"target_distance_m":4.0})).unwrap();
        old.advance(Command {
            valid: false,
            yaw_rad: 0.,
            pitch_rad: 0.,
            distance_m: 0.,
            fire: false,
        })
        .unwrap();
        old.inspect().unwrap();
    }
}
