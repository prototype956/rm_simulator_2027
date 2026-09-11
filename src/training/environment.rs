//! No-render physical backend for the phase-2 transport. This entry exposes actuator/referee
//! feedback, optional synthetic detector frames and separate evaluation truth.
use super::protocol::SettlementReply;
use super::protocol::{CONTROL_DT_NS, Command, Environment};
use super::scenario::{self, Scenario, ScriptedTarget};
use super::{closure, measurements, settlement};
use crate::components::*;
use crate::config::SimulationConfig;
use crate::gimbal_actuator::*;
use crate::robomaster::combat::{
    ledger::CombatLedger, reset::TrainingRound, telemetry::CombatTelemetry, *,
};
use crate::robomaster::prelude::{ArmorPlugins, ArmorRoot, PowerRuneRoot, TechCoreRoot};
use crate::setup::{
    HeadlessScene, ScanOutpost, setup, setup_collision, setup_training_outpost,
    setup_training_power_rune, setup_vehicle,
};
use crate::statistic::ProjectileStatistics;
use crate::systems::{
    ControllerState, cleanup_projectiles, projectile_aerodynamics, setup_projectile,
};
use avian3d::dynamics::ccd::SweptCcdSystems;
use avian3d::prelude::*;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use bevy::render::{RenderPlugin, settings::WgpuSettings};
use bevy::time::TimeUpdateStrategy;
use serde_json::{Value, json};
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::time::{Duration, Instant};
use talos_ipc::GimbalCmd;

const PHYSICS_DT_NS: u64 = 1_000_000;

pub struct PhysicalEnvironment {
    config: SimulationConfig,
    asset_path: PathBuf,
    physics_dt_ns: u64,
    app: Option<App>,
    elapsed_ns: u64,
    previous_damage: u64,
}
impl PhysicalEnvironment {
    pub fn new(config: SimulationConfig, asset_path: PathBuf) -> Self {
        Self {
            config,
            asset_path,
            physics_dt_ns: PHYSICS_DT_NS,
            app: None,
            elapsed_ns: 0,
            previous_damage: 0,
        }
    }
    /// Select one validated process-wide physics step; control stays 10 ms and actuator 1 ms.
    pub fn with_physics_step_ns(mut self, dt_ns: u64) -> Result<Self, String> {
        if !matches!(dt_ns, 500_000 | 1_000_000 | 2_000_000) {
            return Err("physics step must be 500, 1000 or 2000 microseconds".into());
        }
        self.physics_dt_ns = dt_ns;
        Ok(self)
    }
}

fn update_clock(time: Res<Time<Fixed>>, mut clock: ResMut<ActuatorClock>) {
    clock.elapsed = time.elapsed();
}

fn build_app(config: SimulationConfig, asset_path: PathBuf) -> Result<App, String> {
    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(AssetPlugin {
                file_path: asset_path.to_string_lossy().into_owned(),
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
        .insert_resource(HeadlessScene)
        .insert_resource(Time::<Fixed>::from_duration(Duration::from_nanos(
            PHYSICS_DT_NS,
        )))
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
        .init_resource::<CombatLedger>()
        .add_systems(Startup, (setup, setup_projectile))
        .add_observer(setup_vehicle)
        .add_observer(setup_collision)
        .add_observer(setup_training_power_rune)
        .add_observer(setup_training_outpost)
        .add_systems(
            FixedPreUpdate,
            (update_clock, scenario::drive_target, update_gimbal_actuator).chain(),
        )
        .add_systems(FixedUpdate, projectile_aerodynamics)
        .add_systems(FixedLast, scenario::verify_target)
        .add_systems(
            PhysicsSchedule,
            (
                scenario::save_scripted_integration
                    .after(SolverSystems::PostSubstep)
                    .before(SweptCcdSystems),
                scenario::restore_scripted_integration
                    .after(SweptCcdSystems)
                    .before(SolverSystems::Restitution),
            ),
        )
        .add_systems(
            FixedLast,
            cleanup_projectiles.before(telemetry::sample_combat),
        );
    app.finish();
    app.cleanup();
    // Asset loading alone may use host time. No physical ticks run before initialization.
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        app.update();
        let w = app.world_mut();
        let robots = w.query::<&RobotIdentity>().iter(w).count();
        let armors = w.query::<&ArmorRoot>().iter(w).count();
        let muzzles = w.query::<&InfantryLaunchOffset>().iter(w).count();
        let colliders = w.query::<&Collider>().iter(w).count();
        let constructors = w
            .query_filtered::<Entity, Or<(
                With<ColliderConstructor>,
                With<ColliderConstructorHierarchy>,
                With<PreciousCollision>,
            )>>()
            .iter(w)
            .count();
        let ground_ready = w
            .query_filtered::<&Children, With<GroundRoot>>()
            .iter(w)
            .next()
            .is_some();
        let power_ready = w
            .query_filtered::<&Children, With<PowerRuneRoot>>()
            .iter(w)
            .next()
            .is_some();
        let outpost_ready = w
            .query_filtered::<&Children, With<ScanOutpost>>()
            .iter(w)
            .next()
            .is_some();
        let tech_core_ready = w
            .query_filtered::<&Children, With<TechCoreRoot>>()
            .iter(w)
            .next()
            .is_some();
        if power_ready
            && outpost_ready
            && tech_core_ready
            && robots == 2
            && armors == 14
            && muzzles == 2
            && colliders > 10
            && constructors == 0
            && ground_ready
        {
            break;
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "scene initialization timed out: robots={robots}, armors={armors}, muzzles={muzzles}, colliders={colliders}, ground={ground_ready}, power={power_ready}, outpost={outpost_ready}, tech_core={tech_core_ready}"
            ));
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    Ok(app)
}

impl Environment for PhysicalEnvironment {
    fn reset(&mut self, round_id: u64, seed: u64, scenario: &Value) -> Result<Value, String> {
        let scenario = Scenario::parse(scenario)?;
        let measurement_config = scenario.measurements.clone();
        let target_hp = scenario.target_hp;
        let mut config = self.config.clone();
        config.gimbal_actuator.integration_hz = 1000.0;
        // Rebuild transactionally: fresh physics/contact, request, RNG and asset-instance state.
        // Asset-cache reuse is a later throughput optimization, not a partial in-place reset.
        let mut app = build_app(config, self.asset_path.clone())?;
        // Geometry is fully loaded but physics has not advanced. Place both roots on checked
        // support before enabling the clock; the old origin may intersect the central obstacle.
        let prepared = scenario::prepare(app.world_mut(), scenario, seed)?;
        let (initial_yaw, initial_pitch) = prepared.initial_gimbal;
        app.world_mut()
            .entity_mut(prepared.controlled_entity)
            .insert((
                prepared.controlled_initial,
                Position(prepared.controlled_initial.translation),
                Rotation(prepared.controlled_initial.rotation),
            ));
        let mut script = prepared.target;
        if let Some(hp) = target_hp {
            script.report["target_hp_override"] = json!(hp);
        }
        app.world_mut().entity_mut(script.entity).insert((
            script.initial,
            Position(script.initial.translation),
            Rotation(script.initial.rotation),
        ));
        {
            let w = app.world_mut();
            let roots: Vec<_> = w
                .query::<(Entity, &RobotIdentity)>()
                .iter(w)
                .map(|(e, _)| e)
                .collect();
            for e in roots {
                w.entity_mut(e).insert((
                    RigidBody::Kinematic,
                    LinearVelocity::ZERO,
                    AngularVelocity::ZERO,
                ));
            }
        }
        // Synchronize colliders/mass and the imported transforms with one canonical 1 ms tick.
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_nanos(
            PHYSICS_DT_NS,
        )));
        app.update();
        app.insert_resource(script);
        {
            let w = app.world_mut();
            // Establish round-local time after pose synchronization; no host epoch is exposed.
            w.insert_resource(Time::<Fixed>::from_duration(Duration::from_nanos(
                self.physics_dt_ns,
            )));
            w.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_nanos(
                self.physics_dt_ns,
            )));
            w.insert_resource(Time::<Virtual>::default());
            w.insert_resource(Time::<()>::default());
            w.resource_mut::<ActuatorClock>().elapsed = Duration::ZERO;
            // Rebase combat clocks AFTER the final settling/synchronization tick.
            // The synchronization tick must not shift the first round-local cooling deadline.
            for (id, mut old) in w
                .query::<(&RobotIdentity, &mut RobotCombatState)>()
                .iter_mut(w)
            {
                let mut rules = old.rules;
                if id.id == TARGET_ROBOT_ID {
                    if let Some(hp) = target_hp {
                        rules.max_hp = hp;
                    }
                }
                let mut state = RobotCombatState::new(rules);
                state.shooter.mechanics = old.shooter.mechanics;
                state.allowance = old.allowance.clone();
                *old = state;
            }
            let mut round = w.resource_mut::<TrainingRound>();
            round.id = round_id;
            round.started_at = Duration::ZERO;
            round.events.clear();
            w.insert_resource(CombatTelemetry::default());
            w.insert_resource(CombatLedger::default());
            let clock = w.resource::<ActuatorClock>();
            let mut actuator = GimbalActuator::default();
            actuator.initialize(initial_yaw, initial_pitch, clock);
            w.insert_resource(actuator);
            w.resource::<SubscribeAutoAim>()
                .0
                .store(true, std::sync::atomic::Ordering::Release);
            w.run_system_once(update_gimbal_actuator)
                .map_err(|e| e.to_string())?;
            // Make the initialized physical hierarchy visible at t=0 without another tick.
            w.run_system_once(
                bevy::transform::systems::propagate_transforms_for::<With<Controlled>>,
            )
            .map_err(|e| e.to_string())?;
            telemetry::sample_combat(w);
        }
        let visibility = measurements::validate_initial_visibility(&mut app)
            .map_err(|error| format!("invalid seed {seed}: {error}"))?;
        app.world_mut().resource_mut::<ScriptedTarget>().report["initial_visibility"] = visibility;
        if let Some(config) = measurement_config {
            app.insert_resource(measurements::Measurements::new(config, seed));
            measurements::tick(&mut app)?;
        }
        let mut data = inspect_app(&mut app)?;
        measurements::attach(app.world(), &mut data);
        data["previous_round"] = self.app.as_mut().map_or(Value::Null, |old| {
            closure::snapshot(old.world_mut(), self.elapsed_ns, self.previous_damage)
        });
        closure::check_response_size(&data)?;
        measurements::commit(app.world_mut());
        self.app = Some(app);
        self.elapsed_ns = 0;
        self.previous_damage = 0;
        Ok(data)
    }

    fn advance(&mut self, command: Command) -> Result<Value, String> {
        let app = self.app.as_mut().ok_or("reset required")?;
        let w = app.world();
        let clock = w.resource::<ActuatorClock>();
        let round_id = w.resource::<TrainingRound>().id;
        w.resource::<GimbalCommandInbox>().submit(
            GimbalCmd {
                timestamp_ns: clock.timestamp_ns(),
                source_round_id: round_id,
                source_capture_timestamp_ns: clock.timestamp_ns(),
                source_frame_sequence: self.elapsed_ns / CONTROL_DT_NS + 1,
                yaw_deg: command.yaw_rad.to_degrees() as f32,
                pitch_deg: -command.pitch_rad.to_degrees() as f32,
                distance_m: if command.valid {
                    command.distance_m as f32
                } else {
                    -1.0
                },
                fire_advice: u8::from(command.fire),
                ..default()
            },
            clock,
        )?;
        self.step_physics()
    }

    fn end_window(&mut self, max_steps: u64) -> Result<SettlementReply, String> {
        let app = self.app.as_mut().ok_or("reset required")?;
        settlement::close(app.world_mut(), max_steps)?;
        let mut data = inspect_app(app)?;
        data["events"] = json!(app.world().resource::<CombatLedger>().events);
        closure::check_response_size(&data)?;
        app.world_mut()
            .resource_mut::<CombatLedger>()
            .events
            .clear();
        Ok(SettlementReply {
            data,
            finished: settlement::finished(app.world()),
        })
    }

    fn settle(&mut self) -> Result<SettlementReply, String> {
        let data = self.step_physics()?;
        Ok(SettlementReply {
            data,
            finished: settlement::finished(self.app.as_ref().unwrap().world()),
        })
    }

    fn inspect(&mut self) -> Result<Value, String> {
        inspect_app(self.app.as_mut().ok_or("reset required")?)
    }
}

impl PhysicalEnvironment {
    fn step_physics(&mut self) -> Result<Value, String> {
        let app = self.app.as_mut().ok_or("reset required")?;
        for _ in 0..CONTROL_DT_NS / self.physics_dt_ns {
            app.update();
            measurements::tick(app)?;
            if let Some(error) = &app.world().resource::<ScriptedTarget>().failed {
                return Err(error.clone());
            }
        }
        settlement::update(app.world_mut());
        let next_elapsed_ns = self.elapsed_ns + CONTROL_DT_NS;
        let w = app.world_mut();
        if w.resource::<Time<Fixed>>().elapsed().as_nanos() as u64 != next_elapsed_ns {
            return Err("fixed clock drift".into());
        }
        if w.resource::<CombatLedger>().failed {
            return Err("event ledger overflow".into());
        }
        let events = w.resource::<CombatLedger>().events.clone();
        let damage = w
            .query::<(&RobotIdentity, &RobotCombatState)>()
            .iter(w)
            .find(|(id, _)| id.id == CONTROLLED_ROBOT_ID)
            .ok_or("controlled robot missing")?
            .1
            .damage
            .damage_dealt;
        let reward = damage
            .checked_sub(self.previous_damage)
            .ok_or("damage counter decreased")?;
        let mut data = inspect_app(app)?;
        data["events"] = json!(events);
        data["reward_damage"] = json!(reward);
        measurements::attach(app.world(), &mut data);
        closure::check_response_size(&data)?;
        measurements::commit(app.world_mut());
        app.world_mut()
            .resource_mut::<CombatLedger>()
            .events
            .clear();
        self.elapsed_ns = next_elapsed_ns;
        self.previous_damage = damage;
        Ok(data)
    }
}

fn inspect_app(app: &mut App) -> Result<Value, String> {
    let w = app.world_mut();
    let g = *w.resource::<GimbalActuatorTelemetry>();
    let muzzle = w
        .query_filtered::<&GlobalTransform, (With<Controlled>, With<InfantryLaunchOffset>)>()
        .single(w)
        .map_err(|e| e.to_string())?;
    let muzzle_pose = json!({"position_bevy_m":muzzle.translation().to_array(),
            "rotation_xyzw":muzzle.rotation().to_array()});
    let frame = w.resource::<CombatTelemetry>().frame;
    let r = frame.self_referee;
    let mut robots: Vec<_> = w.query::<(&RobotIdentity, &Transform, &RobotCombatState, &LinearVelocity, &AngularVelocity)>().iter(w)
            .map(|(id,t,s,v,a)| json!({"robot_id":id.id.0,"position_bevy_m":t.translation.to_array(),
                "rotation_xyzw":t.rotation.to_array(),"velocity_bevy_m_s":v.0.to_array(),"angular_velocity_bevy_rad_s":a.0.to_array(),"hp":s.life.hp,"actual_shots":s.shooter.actual_shots,
                "rejected_requests":s.shooter.rejected_requests,"damage_dealt":s.damage.damage_dealt,"resources":closure::resources(s)})).collect();
    robots.sort_by_key(|r| r["robot_id"].as_u64());
    let mut projectiles: Vec<_> = w.query::<(&shooting::ProjectileShot, &Position)>().iter(w)
            .map(|(s,p)| json!({"projectile_id":s.id,"request_id":s.request.id,"position_bevy_m":p.0.to_array()})).collect();
    projectiles.sort_by_key(|p| p["projectile_id"].as_u64());
    let settlement = settlement::data(w);
    Ok(json!({
        "settlement":settlement,"visual_frames":[],
        "capabilities":{"physical_step":true,"visual_measurements":w.contains_resource::<measurements::Measurements>(),"scripted_motion":true},
        "feedback":{"valid":g.valid,"timestamp_ns":g.state_timestamp_ns,"yaw_rad":g.actual_yaw_rad,
            "pitch_rad":g.actual_pitch_rad,"yaw_velocity_rad_s":g.yaw_velocity_rad_s,
            "pitch_velocity_rad_s":g.pitch_velocity_rad_s,"command_valid":g.command_valid,
            "consumed_command_timestamp_ns":g.consumed_command_timestamp_ns,
    "mode":g.mode,"consumed_at_timestamp_ns":g.consumed_at_timestamp_ns,
    "target_yaw_rad":g.target_yaw_rad,"target_pitch_rad":g.target_pitch_rad,
    "yaw_acceleration_rad_s2":g.yaw_acceleration_rad_s2,"pitch_acceleration_rad_s2":g.pitch_acceleration_rad_s2,"saturation_flags":g.saturation_flags},
        "self_referee":{"valid":frame.referee_valid != 0,"sample_ns":frame.referee_sample_ns,
            "sequence":frame.referee_sample_sequence,"hp":r.hp,"heat":r.heat,"heat_limit":r.heat_limit,
            "cooling_per_second":r.cooling_per_second,"allowance_mode":r.allowance_mode,
            "allowance_remaining":r.allowance_remaining,"fire_permitted":r.fire_permitted != 0,"fire_blocks":r.fire_blocks},
        "evaluation":{"controlled_muzzle":muzzle_pose,"robots":robots,"projectiles":projectiles,"scenario":w.resource::<ScriptedTarget>().report,
            "motion_diagnostics":{"ccd_motion_corrections":w.resource::<ScriptedTarget>().ccd_motion_corrections,
                "max_ccd_translation_correction_m":w.resource::<ScriptedTarget>().max_ccd_translation_correction_m}},"events":[],"reward_damage":0
    }))
}
