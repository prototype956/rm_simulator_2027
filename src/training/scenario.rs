//! Seeded, bounded training scenarios. Geometry checks use the imported collision world;
//! the conservative envelopes here are reset guards, never replacements for combat colliders.
use crate::components::{GameLayer, GroundRoot};
use crate::robomaster::combat::damage::CombatDead;
use crate::robomaster::combat::{CONTROLLED_ROBOT_ID, RobotIdentity, RobotMember, TARGET_ROBOT_ID};
use avian3d::collision::collider::contact_query;
use avian3d::dynamics::solver::solver_body::SolverBody;
use avian3d::prelude::*;
use bevy::prelude::*;
use rand::{RngExt, SeedableRng};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

const MAX_ATTEMPTS: usize = 64;
const PATH_SPACING: f32 = 0.02;
const FLOOR_VARIATION: f32 = 0.003;
const SUPPORT_GAP: f32 = 0.005;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Motion {
    Static {},
    /// World-X sinusoid about the spawn, initially moving in +X. Speed is the peak speed.
    Translation {
        amplitude_m: f32,
        speed_m_s: f32,
    },
    /// Constant rotation about world +Y, signed right-handed angular speed.
    Rotation {
        angular_speed_rad_s: f32,
    },
}
impl Default for Motion {
    fn default() -> Self {
        Self::Static {}
    }
}
impl Motion {
    fn validate(&self) -> Result<(), String> {
        match *self {
            Self::Static {} => Ok(()),
            Self::Translation { amplitude_m: a, speed_m_s: v }
                if a.is_finite() && a > 0.0 && a <= 2.0 && v.is_finite() && (0.0..=4.0).contains(&v) => Ok(()),
            Self::Rotation { angular_speed_rad_s: w } if w.is_finite() && w.abs() <= 7.0 => Ok(()),
            _ => Err("motion requires amplitude in (0, 2] m, peak speed in [0, 4] m/s, signed angular speed in [-7, 7] rad/s".into()),
        }
    }
    fn excursion(&self) -> f32 {
        match *self {
            Self::Translation {
                amplitude_m,
                speed_m_s,
            } if speed_m_s > 0.0 => amplitude_m,
            _ => 0.0,
        }
    }
    pub fn pose(&self, initial: Transform, time_s: f64) -> Transform {
        let mut t = initial;
        match *self {
            Self::Static {} => {}
            Self::Translation {
                amplitude_m: a,
                speed_m_s: v,
            } => {
                t.translation.x += (a as f64 * (v as f64 / a as f64 * time_s).sin()) as f32;
            }
            Self::Rotation {
                angular_speed_rad_s: w,
            } => {
                let angle = (w as f64 * time_s).rem_euclid(std::f64::consts::TAU) as f32;
                t.rotation = Quat::from_rotation_y(angle) * initial.rotation;
            }
        }
        t
    }
}

#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    /// Training-only standard-target HP override; omitted preserves the combat preset.
    pub target_hp: Option<u32>,
    pub measurements: Option<super::measurements::MeasurementConfig>,
    /// Optional Bevy world X/Z position; omitted samples the imported ground bounds.
    controlled_position_xz_m: Option<[f32; 2]>,
    controlled_yaw_rad: Option<f32>,
    target_distance_m: Option<f32>,
    target_yaw_rad: Option<f32>,
    /// Bevy world +Y bearing from -Z; independent of the target's chassis yaw.
    target_bearing_rad: Option<f32>,
    gimbal_yaw_rad: Option<f64>,
    gimbal_pitch_rad: Option<f64>,
    #[serde(default)]
    motion: Motion,
}
impl Scenario {
    pub fn parse(value: &Value) -> Result<Self, String> {
        let scenario: Self = serde_json::from_value(value.clone()).map_err(|e| e.to_string())?;
        if scenario
            .controlled_position_xz_m
            .is_some_and(|p| p.iter().any(|v| !v.is_finite()))
            || scenario.controlled_yaw_rad.is_some_and(|v| !v.is_finite())
            || scenario
                .target_distance_m
                .is_some_and(|v| !v.is_finite() || !(2.0..=8.0).contains(&v))
            || scenario.target_yaw_rad.is_some_and(|v| !v.is_finite())
            || scenario.target_bearing_rad.is_some_and(|v| !v.is_finite())
            || scenario
                .gimbal_yaw_rad
                .is_some_and(|v| !v.is_finite() || v.abs() > std::f64::consts::PI)
            || scenario.gimbal_pitch_rad.is_some_and(|v| !v.is_finite())
        {
            return Err("scenario requires finite yaw and distance in [2, 8] m".into());
        }
        if scenario
            .target_hp
            .is_some_and(|hp| !(1..=1_000_000).contains(&hp))
        {
            return Err("target_hp must be in [1, 1000000]".into());
        }
        scenario.motion.validate()?;
        if let Some(config) = &scenario.measurements {
            config.validate()?;
        }
        Ok(scenario)
    }
    /// Unspecified world angles face the target. Explicit angles are kept and checked later
    /// against actual camera/armor geometry, so an out-of-view seed is rejected, never corrected.
    pub fn initial_gimbal(&self, direction: Vec3, pitch_limit: f64) -> Result<(f64, f64), String> {
        let yaw = self
            .gimbal_yaw_rad
            .unwrap_or_else(|| (-(direction.x as f64)).atan2(-(direction.z as f64)));
        let pitch = self
            .gimbal_pitch_rad
            .unwrap_or_else(|| (direction.y as f64).atan2(direction.xz().length() as f64));
        if !pitch_limit.is_finite() || pitch_limit <= 0.0 || pitch.abs() > pitch_limit {
            return Err("initial gimbal pitch exceeds the configured mechanical limit".into());
        }
        Ok((yaw, pitch))
    }
}

fn seeded_rng(seed: u64, domain: u64) -> rand::rngs::StdRng {
    rand::rngs::StdRng::seed_from_u64(seed ^ domain)
}

/// Exact extrema for a world-X segment; checking only its endpoints misses a near pass.
fn distance_extrema(dx: f32, dz: f32, excursion: f32) -> (f32, f32) {
    (
        (dx.abs() - excursion).max(0.0).hypot(dz),
        (dx.abs() + excursion).hypot(dz),
    )
}

#[derive(Clone)]
struct Obstacle {
    collider: Collider,
    position: Vec3,
    rotation: Quat,
}
#[derive(Clone, Copy)]
struct Envelope {
    radius: f32,
    bottom: f32,
    top: f32,
}
impl Envelope {
    fn collider(&self, extra_radius: f32) -> Collider {
        Collider::cylinder(self.radius + extra_radius, self.top - self.bottom)
    }
    fn center(&self, root: Vec3) -> Vec3 {
        root + Vec3::Y * (self.top + self.bottom) * 0.5
    }
}

fn robot_envelope(world: &mut World, root: Entity) -> Result<Envelope, String> {
    let pose = *world
        .get::<Transform>(root)
        .ok_or("missing robot transform")?;
    let inverse = pose.rotation.inverse();
    let mut bounds = ColliderAabb::INVALID;
    let mut count = 0;
    for (entity, collider, p, r, member) in world
        .query::<(
            Entity,
            &Collider,
            &Position,
            &Rotation,
            Option<&RobotMember>,
        )>()
        .iter(world)
    {
        if entity != root && !member.is_some_and(|m| m.root == root) {
            continue;
        }
        bounds = bounds.merged(collider.aabb(inverse * (p.0 - pose.translation), inverse * r.0));
        count += 1;
    }
    if count < 5 || !bounds.min.is_finite() || !bounds.max.is_finite() {
        return Err(format!(
            "robot collision geometry incomplete: {count} colliders"
        ));
    }
    let extent = bounds.min.abs().max(bounds.max.abs());
    Ok(Envelope {
        radius: Vec2::new(extent.x, extent.z).length(),
        bottom: bounds.min.y,
        top: bounds.max.y,
    })
}

fn ground_height(obstacles: &[Obstacle], x: f32, z: f32) -> Result<f32, String> {
    let hit = obstacles
        .iter()
        .filter_map(|o| {
            o.collider.cast_ray(
                o.position,
                o.rotation,
                Vec3::new(x, 10.0, z),
                Vec3::NEG_Y,
                20.0,
                false,
            )
        })
        .min_by(|a, b| a.0.total_cmp(&b.0));
    let (distance, normal) = hit.ok_or("no ground support")?;
    if normal.dot(Vec3::Y) < 3.0_f32.to_radians().cos() {
        return Err("support surface is sloped".into());
    }
    Ok(10.0 - distance)
}

fn check_support(obstacles: &[Obstacle], p: Vec3, envelope: Envelope) -> Result<f32, String> {
    let height = ground_height(obstacles, p.x, p.z)?;
    // Center plus a ring: reject ledges/slopes rather than freezing a tilted/floating chassis.
    for i in 0..8 {
        let angle = i as f32 * std::f32::consts::TAU / 8.0;
        let h = ground_height(
            obstacles,
            p.x + envelope.radius * angle.cos(),
            p.z + envelope.radius * angle.sin(),
        )?;
        if (h - height).abs() > FLOOR_VARIATION {
            return Err("uneven support footprint".into());
        }
    }
    Ok(height)
}

fn check_clearance(
    obstacles: &[Obstacle],
    p: Vec3,
    envelope: Envelope,
    extra: f32,
) -> Result<(), String> {
    let shape = envelope.collider(extra);
    let center = envelope.center(p);
    let aabb = shape.aabb(center, Quat::IDENTITY);
    for obstacle in obstacles {
        if !aabb.intersects(&obstacle.collider.aabb(obstacle.position, obstacle.rotation)) {
            continue;
        }
        let contact = contact_query::contact(
            &shape,
            center,
            Quat::IDENTITY,
            &obstacle.collider,
            obstacle.position,
            obstacle.rotation,
            0.0,
        )
        .map_err(|_| "unsupported spawn collision query")?;
        if contact.is_some_and(|c| c.penetration > 0.001) {
            return Err("spawn/path envelope intersects an obstacle".into());
        }
    }
    Ok(())
}

#[derive(Resource)]
pub struct ScriptedTarget {
    pub entity: Entity,
    pub initial: Transform,
    pub motion: Motion,
    pub report: Value,
    pub failed: Option<String>,
    pub(super) integrated_motion: Option<(Vec3, Rotation)>,
    pub ccd_motion_corrections: u64,
    pub max_ccd_translation_correction_m: f32,
}

/// A complete supported initial placement, prepared before any physics tick is allowed.
pub struct PreparedScenario {
    pub controlled_entity: Entity,
    pub controlled_initial: Transform,
    pub target: ScriptedTarget,
    pub initial_gimbal: (f64, f64),
}

fn belongs_to_ground(world: &World, mut entity: Entity) -> bool {
    loop {
        if world.get::<GroundRoot>(entity).is_some() {
            return true;
        }
        let Some(parent) = world.get::<ChildOf>(entity) else {
            return false;
        };
        entity = parent.parent();
    }
}

/// Both roots sample supported, non-intersecting placements from named seed-local streams.
/// Only ground meshes provide support: the restored central obstacle cannot become a roof spawn.
/// Visibility is validated after applying these poses and initializing the actual camera hierarchy.
pub fn prepare(
    world: &mut World,
    scenario: Scenario,
    seed: u64,
) -> Result<PreparedScenario, String> {
    let roots: Vec<_> = world
        .query::<(Entity, &RobotIdentity)>()
        .iter(world)
        .map(|(e, id)| (e, id.id))
        .collect();
    let own = roots
        .iter()
        .find(|(_, id)| *id == CONTROLLED_ROBOT_ID)
        .ok_or("controlled root missing")?
        .0;
    let target = roots
        .iter()
        .find(|(_, id)| *id == TARGET_ROBOT_ID)
        .ok_or("target root missing")?
        .0;
    let environment_layers = GameLayer::environment_collision_layers();
    let geometry: Vec<_> = world
        .query::<(Entity, &Collider, &Position, &Rotation, &CollisionLayers)>()
        .iter(world)
        .filter(|(_, _, _, _, layers)| layers.memberships == environment_layers.memberships)
        .map(|(e, c, p, r, _)| {
            (
                e,
                Obstacle {
                    collider: c.clone(),
                    position: p.0,
                    rotation: r.0,
                },
            )
        })
        .collect();
    let ground: Vec<_> = geometry
        .iter()
        .filter(|(e, _)| belongs_to_ground(world, *e))
        .map(|(_, o)| o.clone())
        .collect();
    if ground.is_empty() {
        return Err("ground colliders missing".into());
    }
    let obstacles: Vec<_> = geometry.into_iter().map(|(_, o)| o).collect();
    let own_envelope = robot_envelope(world, own)?;
    let target_envelope = robot_envelope(world, target)?;
    let bounds = ground.iter().fold(ColliderAabb::INVALID, |a, o| {
        a.merged(o.collider.aabb(o.position, o.rotation))
    });
    let margin = own_envelope.radius;
    let min = bounds.min.xz() + Vec2::splat(margin);
    let max = bounds.max.xz() - Vec2::splat(margin);
    if !min.is_finite() || !max.is_finite() || min.x >= max.x || min.y >= max.y {
        return Err("invalid ground sampling bounds".into());
    }
    let own_yaw = scenario.controlled_yaw_rad.unwrap_or_else(|| {
        seeded_rng(seed, 0x73656c665f796177)
            .random_range(-std::f32::consts::PI..std::f32::consts::PI)
    });
    let mut own_rng = seeded_rng(seed, 0x73656c665f787a30);
    let own_attempts = if scenario.controlled_position_xz_m.is_some() {
        1
    } else {
        MAX_ATTEMPTS
    };
    let mut own_rejected = Vec::new();
    let mut own_pose = None;
    for attempt in 1..=own_attempts {
        let [x, z] = scenario.controlled_position_xz_m.unwrap_or_else(|| {
            [
                own_rng.random_range(min.x..=max.x),
                own_rng.random_range(min.y..=max.y),
            ]
        });
        let mut position = Vec3::new(x, 0.0, z);
        let result = (|| {
            let height = check_support(&ground, position, own_envelope)?;
            position.y = height - own_envelope.bottom + SUPPORT_GAP;
            check_clearance(&obstacles, position, own_envelope, 0.0)
        })();
        match result {
            Ok(()) => {
                own_pose = Some((position, attempt));
                break;
            }
            Err(reason) => {
                own_rejected.push(json!({"attempt":attempt,"position_xz_m":[x,z],"reason":reason}))
            }
        }
    }
    let (own_position, own_attempt) = own_pose.ok_or_else(|| {
        format!(
            "invalid seed {seed}: no legal controlled spawn after {own_attempts} attempts: {}",
            json!(own_rejected)
        )
    })?;
    let controlled_initial =
        Transform::from_translation(own_position).with_rotation(Quat::from_rotation_y(own_yaw));
    let mut obstacles_with_own = obstacles;
    obstacles_with_own.push(Obstacle {
        collider: own_envelope.collider(0.0),
        position: own_envelope.center(own_position),
        rotation: Quat::IDENTITY,
    });
    let mut distance_rng = seeded_rng(seed, 0x64697374616e6365);
    let mut bearing_rng = seeded_rng(seed, 0x62656172696e675f);
    let yaw = scenario.target_yaw_rad.unwrap_or_else(|| {
        seeded_rng(seed, 0x7461726765745f79)
            .random_range(-std::f32::consts::PI..std::f32::consts::PI)
    });
    let mut rejected = Vec::new();
    let attempts = if scenario.target_distance_m.is_some() && scenario.target_bearing_rad.is_some()
    {
        1
    } else {
        MAX_ATTEMPTS
    };
    for attempt in 1..=attempts {
        let distance = scenario
            .target_distance_m
            .unwrap_or_else(|| distance_rng.random_range(2.0..=8.0));
        let bearing = scenario.target_bearing_rad.unwrap_or_else(|| {
            bearing_rng.random_range(-std::f32::consts::PI..std::f32::consts::PI)
        });
        let offset = Quat::from_rotation_y(bearing) * Vec3::NEG_Z * distance;
        let mut position = own_position + offset;
        let result = (|| {
            let height = check_support(&ground, position, target_envelope)?;
            position.y = height - target_envelope.bottom + SUPPORT_GAP;
            let excursion = scenario.motion.excursion();
            let (min_distance, max_distance) = distance_extrema(offset.x, offset.z, excursion);
            if min_distance < 2.0 - 1e-5 || max_distance > 8.0 + 1e-5 {
                return Err("motion leaves the 2-8 m range".into());
            }
            let intervals = ((2.0 * excursion / PATH_SPACING).ceil() as usize).max(1);
            for i in 0..=intervals {
                let offset = -excursion + 2.0 * excursion * i as f32 / intervals as f32;
                let p = position + Vec3::X * offset;
                if (check_support(&ground, p, target_envelope)? - height).abs() > FLOOR_VARIATION {
                    return Err("motion leaves the support plane".into());
                }
                check_clearance(
                    &obstacles_with_own,
                    p,
                    target_envelope,
                    if excursion > 0.0 {
                        PATH_SPACING * 0.5
                    } else {
                        0.0
                    },
                )?;
            }
            Ok::<_, String>(intervals + 1)
        })();
        match result {
            Ok(samples) => {
                let pitch_limit = world.resource::<crate::config::SimulationConfig>().vehicle.gimbal_pitch_limit.max(0.01) as f64;
                let initial_gimbal = scenario.initial_gimbal(position - own_position, pitch_limit)?;
                return Ok(PreparedScenario {
                    controlled_entity: own, controlled_initial, initial_gimbal,
                    target: ScriptedTarget {
                        entity: target,
                        initial: Transform::from_translation(position).with_rotation(Quat::from_rotation_y(yaw)),
                        motion: scenario.motion.clone(), failed: None, integrated_motion: None,
                        ccd_motion_corrections: 0, max_ccd_translation_correction_m: 0.0,
                        report: json!({"validated":true,"attempts":attempt,"rejections":rejected,
                        "sampling_revision":4,"target_distance_m":distance,"target_bearing_rad":bearing,"target_yaw_rad":yaw,"motion":scenario.motion,
                        "controlled_yaw_rad":own_yaw,"controlled_attempts":own_attempt,"controlled_rejections":own_rejected,
                        "controlled_bounds_xz_m":[min.to_array(),max.to_array()],
                        "initial_gimbal":{"yaw_rad":initial_gimbal.0,"pitch_rad":initial_gimbal.1,"pitch_limit_rad":pitch_limit},
                        "path_min_distance_m":distance_extrema(offset.x,offset.z,scenario.motion.excursion()).0,
                        "path_max_distance_m":distance_extrema(offset.x,offset.z,scenario.motion.excursion()).1,
                        "target_position_bevy_m":position.to_array(),"controlled_position_bevy_m":own_position.to_array(),
                        "controlled_tilt_rad":0.0,"controlled_support_gap_m":SUPPORT_GAP,"target_support_gap_m":SUPPORT_GAP,
                        "central_power_rune":"static_unpowered", "outposts":"static", "tech_core":"static", "target_envelope_radius_m":target_envelope.radius,"path_samples":samples,
                        "path_sample_spacing_max_m":PATH_SPACING,"support_height_tolerance_m":FLOOR_VARIATION}),
                    },
                });
            }
            Err(reason) => rejected.push(json!({"attempt":attempt,"distance_m":distance,"bearing_rad":bearing,"reason":reason})),
        }
    }
    Err(format!(
        "invalid seed {seed}: no legal target spawn/path after {attempts} attempts: {}",
        json!(rejected)
    ))
}

/// Physics integrates the velocity; do not also teleport the pose or advance a second clock.
pub fn drive_target(
    mut commands: Commands,
    time: Res<Time<Fixed>>,
    script: Option<Res<ScriptedTarget>>,
    mut roots: Query<(
        &Position,
        &Rotation,
        &ComputedCenterOfMass,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &RigidBody,
        Has<CombatDead>,
    )>,
) {
    let Some(script) = script else {
        return;
    };
    let Ok((position, rotation, com, mut velocity, mut angular, body, dead)) =
        roots.get_mut(script.entity)
    else {
        return;
    };
    if dead {
        // Remove scripted power without teleporting or zeroing momentum. Imported mass,
        // gravity, friction and contacts govern the dead body from this tick onward.
        if *body != RigidBody::Dynamic {
            commands.entity(script.entity).insert(RigidBody::Dynamic);
        }
        return;
    }
    let desired = script.motion.pose(script.initial, time.elapsed_secs_f64());
    // Avian linear velocity belongs to the center of mass, whereas the script moves the root.
    velocity.0 = (desired.translation - position.0 + desired.rotation * com.0 - rotation.0 * com.0)
        / time.delta_secs();
    let mut delta = desired.rotation * rotation.0.inverse();
    if delta.w < 0.0 {
        delta = -delta;
    }
    let (axis, angle) = delta.to_axis_angle();
    angular.0 = axis * angle / time.delta_secs();
}

// Avian 0.7 swept CCD clips BOTH bodies to the projectile's impact time, including
// kinematic targets. Preserve this prescribed target's integrated displacement around CCD;
// projectile CCD/contact handling remains active and other bodies are untouched.
pub fn save_scripted_integration(
    script: Option<ResMut<ScriptedTarget>>,
    bodies: Query<&SolverBody, Without<CombatDead>>,
) {
    let Some(mut script) = script else {
        return;
    };
    script.integrated_motion = bodies
        .get(script.entity)
        .ok()
        .map(|b| (b.delta_position, b.delta_rotation));
}
pub fn restore_scripted_integration(
    script: Option<ResMut<ScriptedTarget>>,
    mut bodies: Query<&mut SolverBody, Without<CombatDead>>,
) {
    let Some(mut script) = script else {
        return;
    };
    if let (Some((position, rotation)), Ok(mut body)) =
        (script.integrated_motion, bodies.get_mut(script.entity))
    {
        let adjustment = body.delta_position.distance(position);
        if adjustment > 1e-7 || 1.0 - body.delta_rotation.0.dot(rotation.0).abs() > 1e-7 {
            script.ccd_motion_corrections += 1;
            script.max_ccd_translation_correction_m =
                script.max_ccd_translation_correction_m.max(adjustment);
        }
        body.delta_position = position;
        body.delta_rotation = rotation;
    }
}

pub fn verify_target(
    time: Res<Time<Fixed>>,
    script: Option<ResMut<ScriptedTarget>>,
    roots: Query<(&Position, &Rotation, Has<CombatDead>)>,
) {
    let Some(mut script) = script else {
        return;
    };
    let Ok((position, rotation, dead)) = roots.get(script.entity) else {
        script.failed = Some("scripted target disappeared".into());
        return;
    };
    if dead {
        return;
    }
    let desired = script.motion.pose(script.initial, time.elapsed_secs_f64());
    if position.0.distance(desired.translation) > 0.0001
        || (1.0 - rotation.0.dot(desired.rotation).abs()) > 0.000001
    {
        script.failed = Some(format!(
            "scripted target diverged at {} s: position={:?}, desired={:?}, rotation={:?}, desired_rotation={:?}",
            time.elapsed_secs_f64(),
            position.0,
            desired.translation,
            rotation.0,
            desired.rotation
        ));
    }
}
