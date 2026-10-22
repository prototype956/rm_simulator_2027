//! Shared GUI/training prescribed motion. These are constrained fire-control proxies;
//! contact-force balancing and learned chassis control are deliberately absent; wheel-leg traversals share the contact controller.
use crate::components::Controlled;
use crate::robomaster::combat::{LifeStatus, RobotCombatState, damage::CombatDead};
use crate::robots::{self, RobotProfile, RobotRig};
use avian3d::dynamics::ccd::SweptCcdSystems;
use avian3d::dynamics::solver::solver_body::SolverBody;
use avian3d::prelude::*;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MotionKind {
    #[default]
    Static,
    Translation,
    Rotation,
    Combined,
    Lift,
    Climb,
    Jump,
    Tunnel,
}
#[derive(Deserialize, Serialize, Clone, Debug)]
#[serde(default, deny_unknown_fields)]
pub struct MotionPlan {
    pub kind: MotionKind,
    pub amplitude_m: f32,
    pub speed_m_s: f32,
    pub angular_speed_rad_s: f32,
}
impl Default for MotionPlan {
    fn default() -> Self {
        Self {
            kind: MotionKind::Static,
            amplitude_m: 0.5,
            speed_m_s: 0.5,
            angular_speed_rad_s: 1.,
        }
    }
}
fn smooth(x: f32) -> f32 {
    let x = x.clamp(0., 1.);
    x * x * (3. - 2. * x)
}
impl MotionPlan {
    pub fn validate(&self, profile: RobotProfile) -> Result<(), String> {
        if !self.amplitude_m.is_finite()
            || !(0.05..=2.).contains(&self.amplitude_m)
            || !self.speed_m_s.is_finite()
            || !(0.05..=2.).contains(&self.speed_m_s)
            || !self.angular_speed_rad_s.is_finite()
            || self.angular_speed_rad_s.abs() > 7.
        {
            return Err(
                "motion requires amplitude 0.05..2 m, speed 0.05..2 m/s, |omega|<=7 rad/s".into(),
            );
        }
        if matches!(
            self.kind,
            MotionKind::Lift | MotionKind::Climb | MotionKind::Jump
        ) && profile != RobotProfile::WheelLeg
        {
            return Err("lift/climb/jump requires wheel_leg".into());
        }
        if self.kind == MotionKind::Tunnel && !profile.compact() {
            return Err("tunnel requires low/folding".into());
        }
        Ok(())
    }
    /// Path is local to the prescribed start. Climb starts 0.7 m outside a 0.2 m
    /// deck's free edge, facing inward; tunnel starts 0.6 m before the bridge centre.
    pub fn state(&self, initial: Transform, time_s: f64) -> MotionState {
        let t = time_s as f32;
        let mut state = MotionState {
            pose: initial,
            fold: 0.,
            lift: 0.,
        };
        match self.kind {
            MotionKind::Static => {}
            MotionKind::Translation | MotionKind::Combined => {
                state.pose.translation += initial.rotation
                    * Vec3::X
                    * (self.amplitude_m * (t * self.speed_m_s / self.amplitude_m).sin());
            }
            MotionKind::Lift => state.lift = 0.1 * (1. - (t * std::f32::consts::TAU / 4.).cos()),
            MotionKind::Climb | MotionKind::Jump => {} // Shared contact/ballistic controller owns this root.
            MotionKind::Tunnel => {
                let u = t.rem_euclid(14.);
                let distance = if u < 1. {
                    0.
                } else if u < 5. {
                    1.3 * smooth((u - 1.) / 4.)
                } else if u < 8. {
                    1.3
                } else if u < 12. {
                    1.3 * (1. - smooth((u - 8.) / 4.))
                } else {
                    0.
                };
                state.pose.translation += initial.rotation * Vec3::X * distance;
                state.fold = if u < 0.1 {
                    0.
                } else if u < 0.9 {
                    (u - 0.1) / 0.8
                } else if u < 5. {
                    1.
                } else if u < 5.8 {
                    1. - (u - 5.) / 0.8
                } else if u < 7. {
                    0.
                } else if u < 7.8 {
                    (u - 7.) / 0.8
                } else if u < 12. {
                    1.
                } else if u < 12.8 {
                    1. - (u - 12.) / 0.8
                } else {
                    0.
                };
            }
            MotionKind::Rotation => {}
        }
        if matches!(self.kind, MotionKind::Rotation | MotionKind::Combined) {
            state.pose.rotation = initial.rotation
                * Quat::from_rotation_y(
                    (t * self.angular_speed_rad_s).rem_euclid(std::f32::consts::TAU),
                );
        }
        state
    }
    pub fn validation_duration(&self) -> f32 {
        match self.kind {
            MotionKind::Static => 0.,
            MotionKind::Lift => 4.,
            MotionKind::Climb => 12.,
            MotionKind::Tunnel => 14.,
            MotionKind::Translation => 45.,
            _ => 45.,
        }
    }
}
#[derive(Clone, Copy)]
pub struct MotionState {
    pub pose: Transform,
    pub fold: f32,
    pub lift: f32,
}
#[derive(Component, Clone)]
pub struct RobotProgram {
    pub initial: Transform,
    pub plan: MotionPlan,
    pub started_s: f64,
    pub failed: Option<String>,
    integrated: Option<(Vec3, Rotation)>,
    pub traversal_requested: bool,
    pub traversal_complete: bool,
}
impl RobotProgram {
    pub fn new(initial: Transform, plan: MotionPlan) -> Self {
        Self {
            initial,
            plan,
            started_s: 0.,
            failed: None,
            integrated: None,
            traversal_requested: false,
            traversal_complete: false,
        }
    }
}

pub struct RobotMotionPlugin;
impl Plugin for RobotMotionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedPreUpdate, drive_programs.before(robots::update_rigs))
            .add_systems(FixedLast, verify_programs)
            .add_systems(
                PhysicsSchedule,
                (
                    save_integration
                        .after(SolverSystems::PostSubstep)
                        .before(SweptCcdSystems),
                    restore_integration
                        .after(SweptCcdSystems)
                        .before(SolverSystems::Restitution),
                ),
            );
    }
}

/// One shared support query with an explicit requested level. Roofs cannot become the floor
/// under the tunnel; ramps/decks/bridge are only selected when the caller requests that level.
pub fn support_height(world: &mut World, point: Vec3, level: f32) -> Result<f32, String> {
    let mut heights = vec![];
    for (name, c, t) in world.query::<(&Name, &Collider, &Transform)>().iter(world) {
        if !matches!(
            name.as_str(),
            "LeagueFloor" | "HighlandDeck" | "BridgeStrip" | "BridgeTransition" | "TunnelAbutment"
        ) {
            continue;
        }
        if let Some((d, n)) = c.cast_ray(
            t.translation,
            t.rotation,
            Vec3::new(point.x, 1., point.z),
            Vec3::NEG_Y,
            2.,
            false,
        ) {
            let h = 1. - d;
            if n.y > 0.85 && (h - level).abs() < 0.003 {
                heights.push(h);
            }
        }
    }
    heights
        .into_iter()
        .next()
        .ok_or_else(|| format!("no support at requested level {level} m"))
}

pub fn validate_programs(
    world: &mut World,
    programs: &[(Entity, RobotProgram)],
) -> Result<usize, String> {
    let saved: Vec<_> = programs
        .iter()
        .map(|(e, _)| {
            (
                *e,
                *world.get::<Transform>(*e).unwrap(),
                world.get::<Position>(*e).copied(),
                world.get::<Rotation>(*e).copied(),
                world.get::<RobotRig>(*e).unwrap().clone(),
                *world.get::<RobotProfile>(*e).unwrap(),
            )
        })
        .collect();
    let mut traversal_samples = 0;
    for (root, p) in programs {
        if matches!(p.plan.kind, MotionKind::Climb | MotionKind::Jump) {
            traversal_samples += crate::wheel_leg_traversal::validate_script(
                world,
                *root,
                p.plan.kind == MotionKind::Jump,
            )?;
        }
    }
    let duration = programs
        .iter()
        .filter(|(_, p)| !matches!(p.plan.kind, MotionKind::Climb | MotionKind::Jump))
        .map(|(_, p)| p.plan.validation_duration())
        .fold(0., f32::max)
        .min(45.);
    let omega = programs
        .iter()
        .map(|(_, p)| p.plan.angular_speed_rad_s.abs())
        .fold(1., f32::max);
    let dt = 0.02_f32.min(0.04 / omega);
    let count = (duration / dt).ceil().max(1.) as usize;
    let result = (|| {
        for index in 0..=count {
            let t = duration as f64 * index as f64 / count as f64;
            for (e, p) in programs {
                if matches!(p.plan.kind, MotionKind::Climb | MotionKind::Jump) {
                    continue;
                }
                let state = p.plan.state(p.initial, t);
                let rig = world.get::<RobotRig>(*e).unwrap().clone();
                let profile = *world.get::<RobotProfile>(*e).unwrap();
                robots::set_mounts(
                    world,
                    profile,
                    &rig,
                    if profile == RobotProfile::Folding {
                        state.fold
                    } else {
                        0.
                    },
                    state.lift,
                );
                world.entity_mut(*e).insert((
                    state.pose,
                    Position(state.pose.translation),
                    Rotation(state.pose.rotation),
                ));
            }
            for (e, p) in programs {
                if matches!(p.plan.kind, MotionKind::Climb | MotionKind::Jump) {
                    continue;
                }
                let state = p.plan.state(p.initial, t);
                robots::check_pose(world, *e, state.pose)
                    .map_err(|r| format!("invalid prescribed path at {t:.3}s: {r}"))?;
                if !matches!(p.plan.kind, MotionKind::Climb) {
                    let profile = *world.get::<RobotProfile>(*e).unwrap();
                    let level = p.initial.translation.y - profile.root_height();
                    for v in [
                        Vec3::ZERO,
                        Vec3::X * 0.21,
                        Vec3::NEG_X * 0.21,
                        Vec3::Z * 0.21,
                        Vec3::NEG_Z * 0.21,
                    ] {
                        support_height(
                            world,
                            state.pose.translation + state.pose.rotation * v,
                            level,
                        )?;
                    }
                }
            }
        }
        Ok(count + 1 + traversal_samples)
    })();
    for (e, t, p, q, rig, profile) in saved {
        world.entity_mut(e).insert(t);
        if let Some(p) = p {
            world.entity_mut(e).insert(p);
        }
        if let Some(q) = q {
            world.entity_mut(e).insert(q);
        }
        robots::set_mounts(world, profile, &rig, rig.fold, rig.lift_m);
    }
    result
}

pub fn drive_programs(world: &mut World) {
    let clock = world.resource::<Time<Fixed>>();
    let time = clock.elapsed_secs_f64();
    let dt = clock.delta_secs();
    let programs: Vec<_> = world
        .query::<(Entity, &RobotProgram)>()
        .iter(world)
        .map(|(e, p)| (e, p.clone()))
        .collect();
    for (root, program) in programs {
        if world
            .get::<RobotCombatState>(root)
            .is_none_or(|s| s.life.status != LifeStatus::Alive)
        {
            world.entity_mut(root).insert((
                RigidBody::Dynamic,
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ));
            continue;
        }
        if matches!(program.plan.kind, MotionKind::Climb | MotionKind::Jump) {
            if program.failed.is_some() {
                continue;
            }
            if crate::wheel_leg_traversal::owns_root(world, root) {
                continue;
            }
            if program.traversal_requested {
                if world
                    .get::<crate::wheel_leg_traversal::TraversalDiagnostics>(root)
                    .is_some_and(|d| d.completed > 0)
                {
                    world
                        .get_mut::<RobotProgram>(root)
                        .unwrap()
                        .traversal_complete = true;
                    world.entity_mut(root).insert((
                        RigidBody::Kinematic,
                        LinearVelocity::ZERO,
                        AngularVelocity::ZERO,
                    ));
                } else {
                    world.get_mut::<RobotProgram>(root).unwrap().failed =
                        Some("traversal interrupted".into());
                }
                continue;
            }
            world.entity_mut(root).insert((
                RigidBody::Kinematic,
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ));
            if program.plan.kind == MotionKind::Climb {
                world.get_mut::<RobotRig>(root).unwrap().requested_lift_m = 0.2;
                if world.get::<RobotRig>(root).unwrap().lift_m < 0.19999 {
                    continue;
                }
            }
            let mode = if program.plan.kind == MotionKind::Jump {
                crate::wheel_leg_traversal::TraversalMode::Jump
            } else {
                crate::wheel_leg_traversal::TraversalMode::RaisedDrive
            };
            match crate::wheel_leg_traversal::request(world, root, mode) {
                Ok(_) => {
                    world
                        .get_mut::<RobotProgram>(root)
                        .unwrap()
                        .traversal_requested = true
                }
                Err(e) => world.get_mut::<RobotProgram>(root).unwrap().failed = Some(e),
            }
            continue;
        }
        if program.failed.is_some() {
            world
                .entity_mut(root)
                .insert((LinearVelocity::ZERO, AngularVelocity::ZERO));
            continue;
        }
        let state = program
            .plan
            .state(program.initial, (time - program.started_s).max(0.));
        let profile = *world.get::<RobotProfile>(root).unwrap();
        let rig = world.get::<RobotRig>(root).unwrap().clone();
        if profile == RobotProfile::Folding {
            let u = ((time - program.started_s) as f32).rem_euclid(14.);
            if program.plan.kind == MotionKind::Tunnel {
                let desired = (0.1..5.).contains(&u) || (7.0..12.).contains(&u);
                if desired != rig.requested_fold {
                    if let Err(reason) = robots::request_fold(world, root, desired) {
                        world.get_mut::<RobotProgram>(root).unwrap().failed = Some(reason);
                        continue;
                    }
                }
            }
        }
        if profile == RobotProfile::WheelLeg {
            world.get_mut::<RobotRig>(root).unwrap().requested_lift_m = state.lift;
        }
        if let Err(reason) = robots::check_pose(world, root, state.pose) {
            world.get_mut::<RobotProgram>(root).unwrap().failed =
                Some(format!("runtime collision: {reason}"));
            world
                .entity_mut(root)
                .insert((LinearVelocity::ZERO, AngularVelocity::ZERO));
            continue;
        }
        let position = world.get::<Position>(root).unwrap().0;
        let rotation = world.get::<Rotation>(root).unwrap().0;
        let com = world
            .get::<ComputedCenterOfMass>(root)
            .map_or(Vec3::ZERO, |c| c.0);
        let linear =
            (state.pose.translation - position + state.pose.rotation * com - rotation * com) / dt;
        let mut delta = state.pose.rotation * rotation.inverse();
        if delta.w < 0. {
            delta = -delta;
        }
        let (axis, angle) = delta.to_axis_angle();
        world.entity_mut(root).insert((
            RigidBody::Kinematic,
            LinearVelocity(linear),
            AngularVelocity(axis * angle / dt),
        ));
    }
}

fn save_integration(mut programs: Query<(&mut RobotProgram, &SolverBody), Without<CombatDead>>) {
    for (mut p, b) in &mut programs {
        p.integrated = Some((b.delta_position, b.delta_rotation));
    }
}
pub fn restore_integration(
    mut programs: Query<(&RobotProgram, &mut SolverBody), Without<CombatDead>>,
) {
    for (p, mut b) in &mut programs {
        if let Some((x, r)) = p.integrated {
            b.delta_position = x;
            b.delta_rotation = r;
        }
    }
}
fn verify_programs(
    time: Res<Time<Fixed>>,
    mut roots: Query<(&Position, &Rotation, &mut RobotProgram), Without<CombatDead>>,
) {
    for (p, r, mut program) in &mut roots {
        if program.failed.is_some()
            || matches!(program.plan.kind, MotionKind::Climb | MotionKind::Jump)
        {
            continue;
        }
        let desired = program
            .plan
            .state(program.initial, time.elapsed_secs_f64() - program.started_s)
            .pose;
        if p.0.distance(desired.translation) > 0.001 || 1. - r.0.dot(desired.rotation).abs() > 1e-5
        {
            program.failed = Some("prescribed root diverged from validated path".into());
        }
    }
}
