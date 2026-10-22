//! Shared wheel-leg edge traversal. Ideal stance/pose control, bounded URDF joints;
//! airborne translation is ballistic. No balance or joint-torque model is claimed.
use crate::components::{ActiveSlapper, Controlled};
use crate::robomaster::combat::{LifeStatus, RobotCombatState};
use crate::{
    robots::{self, RobotProfile, RobotRig},
    wheel_leg_urdf as native,
};
use avian3d::prelude::*;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

const MARGIN: f32 = 0.005;
/// Video cadence approximation, not a calibrated motor rating. Original URDF stays intact.
pub const SIM_JOINT_SPEED_RAD_S: f32 = 25.0;
pub const LIFT_SPEED_M_S: f32 = 1.0;
const APPROACH_SPEED_M_S: f32 = 1.2;
const RAISED_TIME_SCALE: f32 = 0.2;
pub const JUMP_PRELOAD_S: f32 = 0.12;
pub const JUMP_EXTENSION_S: f32 = 0.18;
pub const JUMP_LAUNCH_S: f32 = JUMP_PRELOAD_S + JUMP_EXTENSION_S;
pub const JUMP_SETTLE_S: f32 = 0.10;
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TraversalMode {
    RaisedDrive,
    Jump,
}
#[derive(Clone, Debug)]
pub struct Edge {
    pub point: Vec3,
    pub outward: Vec3,
    pub top: f32,
    pub floor: f32,
    pub tangent_min: f32,
    pub tangent_max: f32,
}
#[derive(Component, Clone, Debug)]
pub struct Traversal {
    pub mode: TraversalMode,
    pub edge: Edge,
    pub initial: Transform,
    pub started: f64,
    pub lift: f32,
    pub approach: f32,
    pub flight: f32,
    pub launch_v: f32,
    pub gravity: f32,
    pub clearance: f32,
    pub duration: f32,
    pub expected: Transform,
}
#[derive(Component, Clone, Debug, Default)]
pub struct TraversalDiagnostics {
    pub phase: String,
    pub reason: Option<String>,
    pub completed: u32,
    pub retry_at: f64,
}
#[derive(Component, Clone, Debug)]
pub struct Recovery {
    started: Option<f64>,
    initial: Transform,
    body: Transform,
    feet: [Vec3; 2],
    goal: Transform,
}
#[derive(Clone, Copy)]
struct Frame {
    root: Transform,
    body: Transform,
    feet: Option<[Vec3; 2]>,
    rise: f32,
    phase: &'static str,
}
fn smooth(x: f32) -> f32 {
    let x = x.clamp(0., 1.);
    x * x * (3. - 2. * x)
}
fn root_pose(world: &World, root: Entity) -> Transform {
    let (_, q, p) = robots::physical_matrix(world, root).to_scale_rotation_translation();
    Transform::from_translation(p).with_rotation(q)
}
pub fn owns_root(world: &World, root: Entity) -> bool {
    world.get::<Traversal>(root).is_some() || world.get::<Recovery>(root).is_some()
}
fn walkable(name: &str) -> bool {
    matches!(
        name,
        "LeagueFloor" | "HighlandDeck" | "BridgeStrip" | "BridgeTransition" | "TunnelAbutment"
    )
}
pub fn floor_height(world: &mut World, p: Vec3, ceiling: f32) -> Option<f32> {
    world
        .query::<(Entity, &Name, &Collider)>()
        .iter(world)
        .filter(|(_, n, _)| walkable(n.as_str()))
        .filter_map(|(e, _, c)| {
            let (_, q, t) = robots::physical_matrix(world, e).to_scale_rotation_translation();
            c.cast_ray(
                t,
                q,
                Vec3::new(p.x, ceiling + 0.02, p.z),
                Vec3::NEG_Y,
                5.,
                false,
            )
            .filter(|(_, n)| n.y > 0.85)
            .map(|(d, _)| ceiling + 0.02 - d)
        })
        .filter(|h| *h <= ceiling + 0.001)
        .max_by(f32::total_cmp)
}
pub fn find_edge(world: &mut World, root: Entity, max_distance: f32) -> Result<Edge, String> {
    let rig = world.get::<RobotRig>(root).ok_or("missing rig")?.clone();
    let pose = root_pose(world, root);
    let heading = robots::physical_matrix(world, rig.base)
        .transform_vector3(Vec3::NEG_Z)
        .with_y(0.)
        .normalize_or_zero();
    let feet = native::wheel_centers(world, &rig, pose);
    let ceiling = feet
        .iter()
        .map(|p| p.y - native::WHEEL_RADIUS + 0.04)
        .fold(f32::NEG_INFINITY, f32::max);
    let floor = floor_height(world, pose.translation, ceiling).ok_or("no support below chassis")?;
    let decks: Vec<_> = world
        .query::<(Entity, &Name, &Collider)>()
        .iter(world)
        .filter(|(_, n, _)| n.as_str() == "HighlandDeck")
        .map(|(e, _, c)| {
            let (_, q, p) = robots::physical_matrix(world, e).to_scale_rotation_translation();
            c.aabb(p, q)
        })
        .collect();
    let mut choices = vec![];
    for b in decks {
        if (b.max.y - floor - 0.2).abs() > 0.003 {
            continue;
        }
        for outward in [Vec3::X, Vec3::NEG_X, Vec3::Z, Vec3::NEG_Z] {
            let point = Vec3::new(
                if outward.x > 0. {
                    b.max.x
                } else if outward.x < 0. {
                    b.min.x
                } else {
                    pose.translation.x
                },
                b.max.y,
                if outward.z > 0. {
                    b.max.z
                } else if outward.z < 0. {
                    b.min.z
                } else {
                    pose.translation.z
                },
            );
            let d = (pose.translation - point).dot(outward);
            let tangent = outward.cross(Vec3::Y);
            let p = pose.translation.dot(tangent);
            let values = [
                Vec3::new(b.min.x, 0., b.min.z).dot(tangent),
                Vec3::new(b.max.x, 0., b.max.z).dot(tangent),
            ];
            let lo = values[0].min(values[1]);
            let hi = values[0].max(values[1]);
            if d > 0.16
                && d <= max_distance
                && p > lo + 0.38
                && p < hi - 0.38
                && heading.dot(-outward) >= 15_f32.to_radians().cos()
            {
                choices.push((
                    d,
                    Edge {
                        point,
                        outward,
                        top: b.max.y,
                        floor,
                        tangent_min: lo,
                        tangent_max: hi,
                    },
                ));
            }
        }
    }
    choices.sort_by(|a, b| a.0.total_cmp(&b.0));
    choices
        .first()
        .map(|(_, e)| e.clone())
        .ok_or_else(|| "align chassis within 15deg of a clear 200mm deck edge".to_owned())
}
fn make(world: &mut World, root: Entity, mode: TraversalMode) -> Result<Traversal, String> {
    let rig = world.get::<RobotRig>(root).ok_or("missing rig")?.clone();
    if rig.urdf.is_none() {
        return Err("wheel_leg only".into());
    }
    if world
        .get::<RobotCombatState>(root)
        .is_none_or(|s| s.life.status != LifeStatus::Alive)
    {
        return Err("dead robot cannot traverse".into());
    }
    if mode == TraversalMode::RaisedDrive
        && (rig.lift_m < 0.19999 || rig.requested_lift_m < 0.19999)
    {
        return Err("raise legs fully with X first".into());
    }
    let edge = find_edge(
        world,
        root,
        if mode == TraversalMode::Jump {
            0.95
        } else {
            0.72
        },
    )?;
    let initial = root_pose(world, root);
    let d = (initial.translation - edge.point).dot(edge.outward);
    if mode == TraversalMode::Jump && d < 0.38 {
        return Err("back away at least 0.38m from the edge before jumping".into());
    }
    let gravity = world.resource::<Gravity>().0;
    if gravity.x.abs() > 0.001
        || gravity.z.abs() > 0.001
        || !gravity.y.is_finite()
        || gravity.y >= 0.
    {
        return Err("traversal requires downward gravity".into());
    }
    let gravity = -gravity.y;
    let feet = native::wheel_centers(world, &rig, initial);
    let clearance = (feet
        .iter()
        .map(|p| p.y - native::WHEEL_RADIUS - edge.floor)
        .sum::<f32>()
        * 0.5)
        .clamp(0., MARGIN);
    for foot in feet {
        if !floor_height(world, foot, foot.y - native::WHEEL_RADIUS + 0.025)
            .is_some_and(|h| (foot.y - native::WHEEL_RADIUS - h).abs() < 0.025)
        {
            return Err("both drive wheels must be supported before traversal".into());
        }
    }
    if world
        .get::<LinearVelocity>(root)
        .is_some_and(|v| v.y.abs() > 0.25)
    {
        return Err("settle before starting traversal".into());
    }
    let flight_v = (2. * gravity * (edge.top - edge.floor + 0.12)).sqrt();
    let flight = (flight_v + (flight_v * flight_v - 2. * gravity * (edge.top - edge.floor)).sqrt())
        / gravity;
    if mode == TraversalMode::Jump
        && (d + 0.45) / flight
            > world
                .resource::<crate::config::SimulationConfig>()
                .vehicle
                .max_speed
    {
        return Err("jump landing requires excessive horizontal speed".into());
    }
    let approach = ((d - 0.32) / APPROACH_SPEED_M_S).max(0.);
    Ok(Traversal {
        mode,
        edge,
        initial,
        started: world.resource::<Time<Fixed>>().elapsed_secs_f64(),
        lift: rig.lift_m,
        approach,
        flight,
        launch_v: flight_v,
        gravity,
        clearance,
        duration: if mode == TraversalMode::Jump {
            JUMP_LAUNCH_S + flight + JUMP_SETTLE_S
        } else {
            approach + (0.5 + 1.8 + 0.9 + 1.2) * RAISED_TIME_SCALE
        },
        expected: initial,
    })
}
fn with_distance(t: &Traversal, d: f32) -> Transform {
    let start = (t.initial.translation - t.edge.point).dot(t.edge.outward);
    Transform {
        translation: t.initial.translation + t.edge.outward * (d - start),
        ..t.initial
    }
}
fn set_body(world: &mut World, rig: &RobotRig, body: Transform) {
    world.entity_mut(rig.body_mount).insert(body);
}
fn wheels(
    world: &World,
    rig: &RobotRig,
    pose: Transform,
    height: f32,
    clearance: f32,
) -> [Vec3; 2] {
    native::rest_wheels(world, rig, pose)
        .map(|p| p.with_y(height + native::WHEEL_RADIUS + clearance))
}
fn body(world: &World, rig: &RobotRig, pitch: f32, y: f32) -> Transform {
    let base = world.get::<Transform>(rig.base).unwrap();
    Transform::from_translation(Vec3::Y * y)
        .with_rotation(Quat::from_axis_angle(base.rotation * Vec3::X, pitch))
}
fn front_support_body(
    world: &World,
    rig: &RobotRig,
    t: &Traversal,
    pose: Transform,
    pitch: f32,
) -> Transform {
    let mut b = body(world, rig, pitch, 0.);
    let base = *world.get::<Transform>(rig.base).unwrap();
    let mut supports = native::auxiliary_supports();
    supports.sort_by(|a, c| {
        let a = pose.rotation * (base.rotation * *a);
        let c = pose.rotation * (base.rotation * *c);
        a.dot(-t.edge.outward).total_cmp(&c.dot(-t.edge.outward))
    });
    let low = supports
        .iter()
        .rev()
        .take(2)
        .map(|p| {
            (pose.to_matrix() * b.to_matrix() * base.to_matrix())
                .transform_point3(*p)
                .y
        })
        .fold(f32::INFINITY, f32::min);
    b.translation.y = t.edge.top + MARGIN - low;
    b
}
fn sample(world: &mut World, rig: &RobotRig, t: &Traversal, time: f32) -> Frame {
    let initial_d = (t.initial.translation - t.edge.point).dot(t.edge.outward);
    if t.mode == TraversalMode::Jump {
        if time < JUMP_PRELOAD_S {
            let y = t.lift * (1. - smooth(time / JUMP_PRELOAD_S));
            let b = body(world, rig, 0., y);
            set_body(world, rig, b);
            return Frame {
                root: t.initial,
                body: b,
                feet: Some(wheels(world, rig, t.initial, t.edge.floor, t.clearance)),
                rise: 0.,
                phase: "jump_preload",
            };
        }
        if time < JUMP_LAUNCH_S {
            let y = 0.2 * smooth((time - JUMP_PRELOAD_S) / JUMP_EXTENSION_S);
            let b = body(world, rig, 0., y);
            set_body(world, rig, b);
            return Frame {
                root: t.initial,
                body: b,
                feet: Some(wheels(world, rig, t.initial, t.edge.floor, t.clearance)),
                rise: 0.,
                phase: "jump_extend",
            };
        }
        let a = time - JUMP_LAUNCH_S;
        if a < t.flight {
            let u = a / t.flight;
            let mut pose = with_distance(t, initial_d - (initial_d + 0.45) * u);
            pose.translation.y += t.launch_v * a - 0.5 * t.gravity * a * a;
            let b = body(world, rig, 0.22 * (std::f32::consts::PI * u).sin(), 0.2);
            return Frame {
                root: pose,
                body: b,
                feet: None,
                rise: 0.2 - 0.06 * (std::f32::consts::PI * u).sin().powi(2),
                phase: "jump_airborne",
            };
        }
        let u = ((a - t.flight) / JUMP_SETTLE_S).clamp(0., 1.);
        let mut pose = with_distance(t, -0.45);
        pose.translation.y += t.edge.top - t.edge.floor;
        let b = body(
            world,
            rig,
            0.,
            0.2 - 0.02 * (std::f32::consts::PI * u).sin(),
        );
        set_body(world, rig, b);
        return Frame {
            root: pose,
            body: b,
            feet: Some(wheels(world, rig, pose, t.edge.top, t.clearance)),
            rise: 0.,
            phase: "jump_settle",
        };
    }
    if time < t.approach {
        let d = initial_d + (0.32 - initial_d) * (time / t.approach).clamp(0., 1.);
        let pose = with_distance(t, d);
        let b = body(world, rig, 0., 0.2);
        set_body(world, rig, b);
        return Frame {
            root: pose,
            body: b,
            feet: Some(wheels(world, rig, pose, t.edge.floor, t.clearance)),
            rise: 0.,
            phase: "raised_approach",
        };
    }
    let a = (time - t.approach) / RAISED_TIME_SCALE;
    if a < 0.5 {
        let u = smooth(a / 0.5);
        let pose = with_distance(t, 0.32 - 0.14 * u);
        let contact = front_support_body(world, rig, t, pose, -0.23 * u);
        let mut b = body(
            world,
            rig,
            -0.23 * u,
            0.2 + (contact.translation.y - 0.2) * u,
        );
        if u > 0.999 {
            b = contact;
        }
        set_body(world, rig, b);
        return Frame {
            root: pose,
            body: b,
            feet: Some(wheels(world, rig, pose, t.edge.floor, t.clearance)),
            rise: 0.,
            phase: "front_support",
        };
    }
    if a < 2.3 {
        let u = smooth((a - 0.5) / 1.8);
        let pose = with_distance(t, 0.18 - 0.33 * u);
        let b = front_support_body(world, rig, t, pose, -0.23);
        set_body(world, rig, b);
        let mut targets = wheels(
            world,
            rig,
            pose,
            t.edge.floor + (t.edge.top - t.edge.floor) * smooth(u / 0.55),
            t.clearance,
        );
        let start_pose = with_distance(t, 0.18);
        let delta = pose.translation - start_pose.translation;
        for p in &mut targets {
            let nominal_d = (*p - t.edge.point).dot(t.edge.outward);
            let start_d = nominal_d - delta.dot(t.edge.outward);
            let d = start_d + (nominal_d - start_d) * smooth((u - 0.55) / 0.45);
            let heading =
                (pose.rotation * world.get::<Transform>(rig.base).unwrap().rotation * Vec3::NEG_Z)
                    .with_y(0.)
                    .normalize();
            *p += heading * ((d - nominal_d) / heading.dot(t.edge.outward));
        }
        return Frame {
            root: pose,
            body: b,
            feet: Some(targets),
            rise: 0.,
            phase: "wheel_transfer",
        };
    }
    if a < 3.2 {
        let u = smooth((a - 2.3) / 0.9);
        let pose = with_distance(t, -0.15 - 0.30 * u);
        let contact = front_support_body(world, rig, t, pose, -0.23 * (1. - u));
        let b = body(
            world,
            rig,
            -0.23 * (1. - u),
            contact.translation.y + (0.2 - contact.translation.y) * u,
        );
        set_body(world, rig, b);
        return Frame {
            root: pose,
            body: b,
            feet: Some(wheels(world, rig, pose, t.edge.top, t.clearance)),
            rise: 0.,
            phase: "roll_onto_deck",
        };
    }
    let u = smooth((a - 3.2) / 1.2);
    let mut pose = with_distance(t, -0.45);
    pose.translation.y += (t.edge.top - t.edge.floor) * u;
    let b = body(world, rig, 0., 0.2);
    set_body(world, rig, b);
    Frame {
        root: pose,
        body: b,
        feet: Some(wheels(world, rig, pose, t.edge.top, t.clearance)),
        rise: 0.,
        phase: "raised_settle",
    }
}
fn apply(world: &mut World, rig: &RobotRig, f: Frame) -> Result<(), String> {
    set_body(world, rig, f.body);
    if let Some(targets) = f.feet {
        native::set_world_wheels(world, rig, f.root, targets)
    } else {
        native::apply_legs(world, rig.urdf.as_ref().unwrap(), [f.rise; 2])
    }
}
fn angles(world: &World, rig: &RobotRig) -> Vec<Quat> {
    rig.urdf
        .as_ref()
        .unwrap()
        .chains
        .iter()
        .flatten()
        .map(|e| world.get::<Transform>(*e).unwrap().rotation)
        .collect()
}
/// Non-mutating admission from the caller's perspective: restore every sampled transform.
pub fn validate(world: &mut World, root: Entity, t: &Traversal) -> Result<usize, String> {
    let rig = world.get::<RobotRig>(root).unwrap().clone();
    let mut entities = vec![rig.body_mount];
    entities.extend(rig.urdf.as_ref().unwrap().chains.iter().flatten().copied());
    let saved: Vec<_> = entities
        .iter()
        .map(|e| (*e, *world.get::<Transform>(*e).unwrap()))
        .collect();
    let count = (t.duration / 0.005).ceil() as usize;
    let dt = t.duration / count as f32;
    let mut previous = None;
    let result = (|| {
        for i in 0..=count {
            let f = sample(world, &rig, t, i as f32 * dt);
            apply(world, &rig, f).map_err(|e| format!("{} {:.3}s: {e}", f.phase, i as f32 * dt))?;
            let q = angles(world, &rig);
            if let Some(old) = previous {
                let old: Vec<Quat> = old;
                for (a, b) in old.iter().zip(&q) {
                    if a.angle_between(*b) > SIM_JOINT_SPEED_RAD_S * 1.01 * dt {
                        return Err(format!(
                            "joint speed {:.2} exceeds video-profile simulation limit in {} at {:.3}s",
                            a.angle_between(*b) / dt,
                            f.phase,
                            i as f32 * dt
                        ));
                    }
                }
            }
            previous = Some(q);
            robots::check_pose(world, root, f.root)
                .map_err(|e| format!("{} {:.3}s: {e}", f.phase, i as f32 * dt))?;
        }
        let f = sample(world, &rig, t, t.duration);
        apply(world, &rig, f)?;
        let feet = native::wheel_centers(world, &rig, f.root);
        for p in feet {
            if !floor_height(world, p, t.edge.top + 0.03)
                .is_some_and(|h| (h - t.edge.top).abs() < 0.003)
            {
                return Err("landing cannot support both drive wheels".into());
            }
        }
        let heading =
            (f.root.rotation * world.get::<Transform>(rig.base).unwrap().rotation * Vec3::NEG_Z)
                .with_y(0.)
                .normalize();
        for dx in [-0.34, 0.34] {
            for dz in [-0.34, 0.34] {
                let p = f.root.translation + heading * dz + heading.cross(Vec3::Y) * dx;
                if !floor_height(world, p, t.edge.top + 0.03)
                    .is_some_and(|h| (h - t.edge.top).abs() < 0.003)
                {
                    return Err("landing footprint lacks edge clearance".into());
                }
            }
        }
        Ok(count + 1)
    })();
    for (e, p) in saved {
        world.entity_mut(e).insert(p);
    }
    world.entity_mut(root).insert(native::UrdfStatus::default());
    result
}
pub fn request(world: &mut World, root: Entity, mode: TraversalMode) -> Result<usize, String> {
    if owns_root(world, root) {
        return Err("traversal already active".into());
    }
    if world.get::<TraversalDiagnostics>(root).is_none() {
        world
            .entity_mut(root)
            .insert(TraversalDiagnostics::default());
    }
    let t = make(world, root, mode)?;
    let count = validate(world, root, &t)?;
    world.entity_mut(root).insert((
        t,
        RigidBody::Kinematic,
        LinearVelocity::ZERO,
        AngularVelocity::ZERO,
    ));
    if world.get::<TraversalDiagnostics>(root).is_none() {
        world
            .entity_mut(root)
            .insert(TraversalDiagnostics::default());
    }
    let mut rig = world.get_mut::<RobotRig>(root).unwrap();
    rig.requested_lift_m = 0.2;
    rig.blocked = None;
    Ok(count)
}
fn desired_drive(world: &World, root: Entity, rig: &RobotRig) -> Vec3 {
    let Some(c) = world.get_resource::<crate::systems::ControllerState>() else {
        return Vec3::ZERO;
    };
    let controlled = world.get::<Controlled>(root).is_some();
    let input = if controlled {
        c.controlled.movement
    } else if world.get::<ActiveSlapper>(root).is_some() {
        c.remote.movement
    } else {
        return Vec3::ZERO;
    };
    let m = if controlled {
        robots::physical_matrix(world, rig.gimbal)
    } else {
        robots::physical_matrix(world, root)
    };
    (m.transform_vector3(Vec3::NEG_Z)
        .with_y(0.)
        .normalize_or_zero()
        * input.y
        + m.transform_vector3(Vec3::X).with_y(0.).normalize_or_zero() * input.x)
        .normalize_or_zero()
}
fn recover(world: &mut World, root: Entity, rig: &RobotRig, reason: String) {
    let pose = root_pose(world, root);
    let body = *world.get::<Transform>(rig.body_mount).unwrap();
    let feet = native::wheel_centers(world, rig, pose);
    // Preserve physical momentum; gravity continues during the unsupported part.
    world.entity_mut(root).remove::<Traversal>().insert((
        RigidBody::Dynamic,
        Recovery {
            started: None,
            initial: pose,
            body,
            feet,
            goal: pose,
        },
        native::UrdfStatus::default(),
    ));
    world.get_mut::<RobotRig>(root).unwrap().blocked = Some(reason.clone());
    let mut d = world.get_mut::<TraversalDiagnostics>(root).unwrap();
    d.phase = "gravity_recovery".into();
    d.reason = Some(reason);
}
fn recovery(world: &mut World, root: Entity, rig: &RobotRig, mut r: Recovery, now: f64, dt: f32) {
    let current = root_pose(world, root);
    if r.started.is_none() {
        let feet = native::wheel_centers(world, rig, current);
        let levels: Vec<_> = feet
            .iter()
            .filter_map(|p| floor_height(world, *p, p.y - native::WHEEL_RADIUS + 0.03))
            .collect();
        if levels.len() != 2
            || feet
                .iter()
                .zip(&levels)
                .any(|(p, h)| (p.y - native::WHEEL_RADIUS - h).abs() > 0.025)
        {
            return;
        }
        if (levels[0] - levels[1]).abs() > 0.005 {
            return;
        }
        r.started = Some(now);
        r.initial = current;
        r.body = *world.get::<Transform>(rig.body_mount).unwrap();
        r.feet = [0, 1].map(|i| feet[i].with_y(levels[i] + native::WHEEL_RADIUS + MARGIN));
        r.goal = current;
        r.goal.translation.y = levels[0] + RobotProfile::WheelLeg.root_height();
    }
    let u = smooth(((now - r.started.unwrap()) as f32) / 2.0);
    let pose = Transform {
        translation: r.initial.translation.lerp(r.goal.translation, u),
        ..r.initial
    };
    let b = Transform {
        translation: r.body.translation.lerp(Vec3::Y * 0.2, u),
        rotation: r.body.rotation.slerp(Quat::IDENTITY, u),
        ..r.body
    };
    let old = *world.get::<Transform>(rig.body_mount).unwrap();
    let joints: Vec<_> = rig
        .urdf
        .as_ref()
        .unwrap()
        .chains
        .iter()
        .flatten()
        .map(|e| (*e, *world.get::<Transform>(*e).unwrap()))
        .collect();
    set_body(world, rig, b);
    if native::set_world_wheels(world, rig, pose, r.feet)
        .and_then(|_| robots::check_pose(world, root, pose))
        .is_err()
    {
        set_body(world, rig, old);
        for (e, p) in joints {
            world.entity_mut(e).insert(p);
        }
        r.started = None;
        world.entity_mut(root).insert((r, RigidBody::Dynamic));
        return;
    }
    world.entity_mut(root).insert((
        RigidBody::Kinematic,
        LinearVelocity((pose.translation - current.translation) / dt),
        AngularVelocity::ZERO,
        r,
    ));
    world.get_mut::<TraversalDiagnostics>(root).unwrap().phase = "supported_recovery".into();
    if u >= 1. {
        world
            .entity_mut(root)
            .remove::<Recovery>()
            .insert((RigidBody::Dynamic, LinearVelocity::ZERO));
        let mut rig = world.get_mut::<RobotRig>(root).unwrap();
        rig.lift_m = 0.2;
        rig.requested_lift_m = 0.2;
        rig.lift_velocity = 0.;
    }
}
pub fn update(world: &mut World) {
    let now = world.resource::<Time<Fixed>>().elapsed_secs_f64();
    let dt = world.resource::<Time<Fixed>>().delta_secs();
    if dt <= 0. {
        return;
    }
    let roots: Vec<_> = world
        .query::<(Entity, &RobotRig)>()
        .iter(world)
        .filter(|(_, r)| r.urdf.is_some())
        .map(|(e, r)| (e, r.clone()))
        .collect();
    for (root, rig) in roots {
        let alive = world
            .get::<RobotCombatState>(root)
            .is_some_and(|s| s.life.status == LifeStatus::Alive);
        if !alive {
            if owns_root(world, root) {
                world
                    .entity_mut(root)
                    .remove::<(Traversal, Recovery)>()
                    .insert(RigidBody::Dynamic);
            }
            continue;
        }
        if let Some(r) = world.get::<Recovery>(root).cloned() {
            recovery(world, root, &rig, r, now, dt);
            continue;
        }
        if let Some(mut t) = world.get::<Traversal>(root).cloned() {
            let elapsed = (now - t.started) as f32;
            if t.mode == TraversalMode::RaisedDrive
                && elapsed < t.approach
                && world
                    .get::<crate::robot_motion::RobotProgram>(root)
                    .is_none()
                && desired_drive(world, root, &rig).dot(-t.edge.outward) < 0.707
            {
                world.entity_mut(root).remove::<Traversal>().insert((
                    RigidBody::Dynamic,
                    LinearVelocity::ZERO,
                    AngularVelocity::ZERO,
                ));
                world.get_mut::<TraversalDiagnostics>(root).unwrap().phase = "idle".into();
                continue;
            }
            let old_body = *world.get::<Transform>(rig.body_mount).unwrap();
            let old_joints: Vec<_> = rig
                .urdf
                .as_ref()
                .unwrap()
                .chains
                .iter()
                .flatten()
                .map(|e| (*e, *world.get::<Transform>(*e).unwrap()))
                .collect();
            let f = sample(world, &rig, &t, elapsed.min(t.duration));
            let result = apply(world, &rig, f)
                .and_then(|_| robots::check_pose(world, root, f.root))
                .and_then(|_| {
                    if f.phase == "jump_settle" {
                        for p in native::wheel_centers(world, &rig, f.root) {
                            if !floor_height(world, p, t.edge.top + 0.03)
                                .is_some_and(|h| (h - t.edge.top).abs() < 0.003)
                            {
                                return Err("landing support disappeared during flight".into());
                            }
                        }
                    }
                    Ok(())
                });
            if let Err(reason) = result {
                world.entity_mut(rig.body_mount).insert(old_body);
                for (e, p) in old_joints {
                    world.entity_mut(e).insert(p);
                }
                recover(world, root, &rig, reason);
                continue;
            }
            let current = root_pose(world, root);
            let linear = (f.root.translation - current.translation) / dt;
            world.entity_mut(root).insert((
                RigidBody::Kinematic,
                LinearVelocity(linear),
                AngularVelocity::ZERO,
            ));
            t.expected = f.root;
            world.entity_mut(root).insert(t.clone());
            {
                let mut d = world.get_mut::<TraversalDiagnostics>(root).unwrap();
                d.phase = f.phase.into();
                d.reason = None;
            }
            if elapsed >= t.duration {
                let feet = native::wheel_centers(world, &rig, f.root);
                let supported = feet.iter().all(|p| {
                    floor_height(world, *p, t.edge.top + 0.03).is_some_and(|h| {
                        (h - t.edge.top).abs() < 0.003
                            && (p.y - native::WHEEL_RADIUS - h).abs() < 0.02
                    })
                });
                if !supported {
                    recover(
                        world,
                        root,
                        &rig,
                        "landing support lost; gravity recovery".into(),
                    );
                    continue;
                }
                world
                    .entity_mut(root)
                    .insert((
                        f.root,
                        Position(f.root.translation),
                        Rotation(f.root.rotation),
                        LinearVelocity::ZERO,
                        AngularVelocity::ZERO,
                    ))
                    .remove::<Traversal>();
                // Scripted robots remain kinematic at their completed goal; manual driving resumes.
                let body_kind = if world
                    .get::<crate::robot_motion::RobotProgram>(root)
                    .is_some()
                {
                    RigidBody::Kinematic
                } else {
                    RigidBody::Dynamic
                };
                world.entity_mut(root).insert(body_kind);
                let mut r = world.get_mut::<RobotRig>(root).unwrap();
                r.lift_m = 0.2;
                r.requested_lift_m = 0.2;
                r.lift_velocity = 0.;
                r.blocked = None;
                let mut d = world.get_mut::<TraversalDiagnostics>(root).unwrap();
                d.completed += 1;
                d.phase = "idle".into();
            }
        } else if world
            .get::<crate::robot_motion::RobotProgram>(root)
            .is_none()
            && rig.lift_m >= 0.19999
            && world
                .get::<TraversalDiagnostics>(root)
                .is_none_or(|d| now >= d.retry_at)
        {
            if let Ok(edge) = find_edge(world, root, 0.72) {
                if desired_drive(world, root, &rig).dot(-edge.outward) > 0.707 {
                    if let Some(mut v) = world.get_mut::<LinearVelocity>(root) {
                        let horizontal = v.0.with_y(0.);
                        if horizontal.length() > APPROACH_SPEED_M_S {
                            v.0 = horizontal.normalize() * APPROACH_SPEED_M_S + Vec3::Y * v.y;
                        }
                    }
                    if let Err(reason) = request(world, root, TraversalMode::RaisedDrive) {
                        world.get_mut::<RobotRig>(root).unwrap().blocked = Some(reason.clone());
                        let mut d = world.get_mut::<TraversalDiagnostics>(root).unwrap();
                        d.reason = Some(reason);
                        d.retry_at = now + 0.5;
                    }
                }
            }
        }
    }
}
pub fn jump_key(world: &mut World) {
    let now = world.resource::<Time<Fixed>>().elapsed_secs_f64();
    let Some(keys) = world.get_resource::<ButtonInput<KeyCode>>() else {
        return;
    };
    if !keys.just_pressed(KeyCode::F6) {
        return;
    }
    let remote = keys.pressed(KeyCode::AltLeft) || keys.pressed(KeyCode::AltRight);
    let roots: Vec<_> = world
        .query::<(Entity, &RobotRig, Has<Controlled>, Has<ActiveSlapper>)>()
        .iter(world)
        .filter(|(_, r, c, a)| r.urdf.is_some() && if remote { *a } else { *c })
        .map(|(e, _, _, _)| e)
        .collect();
    for root in roots {
        if let Err(reason) = request(world, root, TraversalMode::Jump) {
            world.get_mut::<RobotRig>(root).unwrap().blocked = Some(reason.clone());
            let mut d = world.get_mut::<TraversalDiagnostics>(root).unwrap();
            d.reason = Some(reason);
            d.retry_at = now + 0.5;
        }
    }
}
pub fn reset(world: &mut World) {
    let roots: Vec<_> = world
        .query_filtered::<Entity, With<RobotRig>>()
        .iter(world)
        .collect();
    for root in roots {
        world
            .entity_mut(root)
            .remove::<(Traversal, Recovery)>()
            .insert(TraversalDiagnostics::default());
    }
}
pub fn validate_script(world: &mut World, root: Entity, jump: bool) -> Result<usize, String> {
    let rig = world.get::<RobotRig>(root).unwrap().clone();
    let mut entities = vec![rig.body_mount];
    entities.extend(
        rig.urdf
            .as_ref()
            .ok_or("wheel_leg required")?
            .chains
            .iter()
            .flatten()
            .copied(),
    );
    let saved: Vec<_> = entities
        .iter()
        .map(|e| (*e, *world.get::<Transform>(*e).unwrap()))
        .collect();
    if !jump {
        let mut r = rig.clone();
        r.lift_m = 0.2;
        r.requested_lift_m = 0.2;
        robots::set_mounts(world, RobotProfile::WheelLeg, &r, 0., 0.2);
        world.entity_mut(root).insert(r);
    }
    let result = make(
        world,
        root,
        if jump {
            TraversalMode::Jump
        } else {
            TraversalMode::RaisedDrive
        },
    )
    .and_then(|t| validate(world, root, &t));
    for (e, p) in saved {
        world.entity_mut(e).insert(p);
    }
    world
        .entity_mut(root)
        .insert((rig, native::UrdfStatus::default()));
    result
}
