//! Synthetic detector outputs from shared real-asset geometry. No target truth enters frames.
use crate::gimbal_actuator::{ActuatorClock, GimbalActuatorTelemetry};
use crate::robomaster::{
    combat::*,
    prelude::{ArmorParts, ArmorRoot, VertexData},
};
use crate::{capture_geometry::*, components::*, config::SimulationConfig};
use avian3d::prelude::*;
use bevy::{ecs::system::RunSystemOnce, prelude::*};
use rand::{RngExt, SeedableRng};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::VecDeque;
use talos_ipc::RigidTransformF32;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct MeasurementConfig {
    pub noise_std_px: f64,
    pub latency_ms: u64,
    pub dropout_probability: f64,
    /// Deterministic detector-blackout intervals [start,end), in round-local milliseconds.
    pub blackouts_ms: Vec<[u64; 2]>,
}
impl Default for MeasurementConfig {
    fn default() -> Self {
        Self {
            noise_std_px: 0.25,
            latency_ms: 20,
            dropout_probability: 0.0,
            blackouts_ms: vec![],
        }
    }
}
impl MeasurementConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.noise_std_px.is_finite()
            || !(0.0..=10.0).contains(&self.noise_std_px)
            || self.latency_ms > 500
            || !self.dropout_probability.is_finite()
            || !(0.0..=1.0).contains(&self.dropout_probability)
            || self.blackouts_ms.len() > 16
            || self
                .blackouts_ms
                .iter()
                .any(|p| p[0] >= p[1] || p[1] > 3_600_000)
        {
            return Err("invalid measurement noise/latency/dropout/blackout parameters".into());
        }
        Ok(())
    }
}

#[derive(Resource)]
pub(super) struct Measurements {
    config: MeasurementConfig,
    rng: rand::rngs::StdRng,
    next_index: u64,
    pending: VecDeque<(u64, Value, Value)>,
    pub frames: Vec<Value>,
    pub truth: Vec<Value>,
}
impl Measurements {
    pub fn new(config: MeasurementConfig, seed: u64) -> Self {
        Self {
            config,
            rng: rand::rngs::StdRng::seed_from_u64(seed ^ 0x56495355414c_3030),
            next_index: 0,
            pending: VecDeque::new(),
            frames: vec![],
            truth: vec![],
        }
    }
}

fn transform_data(t: RigidTransformF32) -> Value {
    json!({"translation":t.translation,"rotation_xyzw":[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]})
}

fn capture(
    config: Res<SimulationConfig>,
    clock: Res<ActuatorClock>,
    g: Res<GimbalActuatorTelemetry>,
    round: Res<reset::TrainingRound>,
    robots: Query<(Entity, &GlobalTransform, &Infantry, Option<&RobotIdentity>)>,
    chassis: Query<(&GlobalTransform, &InfantryChassis)>,
    armors: Query<(Entity, &ArmorRoot, &ArmorParts)>,
    vertices: Query<(&GlobalTransform, &VertexData)>,
    parents: Query<&ChildOf>,
    children: Query<&Children>,
    names: Query<&Name>,
    globals: Query<&GlobalTransform>,
    local: Query<&Transform>,
    roles: Query<
        (
            Entity,
            Option<&InfantryGimbal>,
            Option<&InfantryViewOffset>,
            Option<&InfantryLaunchOffset>,
        ),
        With<Controlled>,
    >,
    colliders: Query<(
        &Collider,
        &GlobalTransform,
        Option<&RobotMember>,
        Option<&Name>,
        Option<&RobotIdentity>,
    )>,
) -> Result<(Value, Vec<Value>), String> {
    let (root, _, _, _) = robots
        .iter()
        .find(|(_, _, _, id)| id.is_some_and(|id| id.id == CONTROLLED_ROBOT_ID))
        .ok_or("controlled root missing")?;
    let mut gim = None;
    let mut view = None;
    let mut muzzle = None;
    for (e, a, b, c) in &roles {
        if a.is_some() {
            gim = Some(e);
        }
        if b.is_some() {
            view = Some(e);
        }
        if c.is_some() {
            muzzle = Some(e);
        }
    }
    let gim = gim.ok_or("gimbal missing")?;
    let view = view.ok_or("camera view offset missing")?;
    let muzzle = muzzle.ok_or("muzzle missing")?;
    let root_t = local.get(root).map_err(|e| e.to_string())?;
    let gim_t = local.get(gim).map_err(|e| e.to_string())?;
    let view_t = local.get(view).map_err(|e| e.to_string())?;
    let muzzle_t = local.get(muzzle).map_err(|e| e.to_string())?;
    let camera = crate::systems::robot_camera_pose(root_t, gim_t, view_t, muzzle_t);
    let camera_rot = camera.rotation;
    let camera_pos = camera.translation;
    let gim_global = globals.get(gim).map_err(|e| e.to_string())?;
    let shot_rot = gim_global.rotation() * muzzle_t.rotation;
    let world_gimbal = transform_from_axes(
        to_ros_translation(gim_global.translation()),
        to_ros_translation(shot_rot * Vec3::Y),
        to_ros_translation(shot_rot * -Vec3::X),
        to_ros_translation(shot_rot * Vec3::Z),
    );
    let world_camera = transform_from_axes(
        to_ros_translation(camera_pos),
        to_ros_translation(camera_rot * Vec3::X),
        to_ros_translation(camera_rot * -Vec3::Y),
        to_ros_translation(camera_rot * -Vec3::Z),
    );
    let muzzle_pos = to_ros_translation(
        globals
            .get(muzzle)
            .map_err(|e| e.to_string())?
            .translation(),
    );
    let mut local_muzzle = RigidTransformF32::default();
    local_muzzle.rotation.w = 1.0;
    local_muzzle.translation = (quat_from_wire(world_gimbal.rotation).inverse()
        * (muzzle_pos - Vec3::from_array(world_gimbal.translation)))
    .to_array();
    let k = crate::capture::compute_camera_intrinsics(
        config.capture.color.width,
        config.capture.color.height,
        config.camera.fov.to_radians(),
    );
    let truth = capture_ground_truth(
        &robots,
        &chassis,
        &armors,
        &vertices,
        &parents,
        &children,
        &names,
        &globals,
        0,
        clock.timestamp_ns(),
    );
    let mut plates: Vec<_> = truth.armors[..truth.armor_count as usize]
        .iter()
        .filter(|a| a.owner_robot_id == TARGET_ROBOT_ID.0)
        .collect();
    plates.sort_by(|a, b| {
        a.world_t_armor
            .translation
            .partial_cmp(&b.world_t_armor.translation)
            .unwrap()
    });
    if plates.len() != 4 {
        return Err(format!(
            "measurement geometry requires four target plates, got {}",
            plates.len()
        ));
    }
    let camera_inv = quat_from_wire(world_camera.rotation).inverse();
    let mut detections = Vec::new();
    let mut evaluation = Vec::new();
    for plate in plates {
        let center = Vec3::from_array(plate.world_t_armor.translation);
        let camera_center = Vec3::from_array(world_camera.translation);
        let normal = quat_from_wire(plate.world_t_armor.rotation) * Vec3::Z;
        let mut reason = "visible";
        let mut obstruction = Value::Null;
        if normal.dot((camera_center - center).normalize()) < 0.1 {
            reason = "back_facing";
        }
        let mut pixels = Vec::new();
        for corner in plate.corners_world {
            let p = camera_inv * (Vec3::from_array(corner) - camera_center);
            if p.z <= 0.1 {
                reason = "near_plane";
            }
            let u = k.fx * p.x as f64 / p.z as f64 + k.cx;
            let v = k.fy * p.y as f64 / p.z as f64 + k.cy;
            if !u.is_finite()
                || !v.is_finite()
                || u < 0.0
                || v < 0.0
                || u >= k.width as f64
                || v >= k.height as f64
            {
                reason = "outside_fov";
            }
            pixels.push([u, v]);
        }
        if reason == "visible" {
            // Conservative collision-geometry visibility: require center and all four corners.
            // The final centimeter tolerates the difference between ideal plate and collider skin.
            for point in plate.corners_world.into_iter().chain([center.to_array()]) {
                let end = ALIGN.transpose() * Vec3::from_array(point);
                let delta = end - camera_pos;
                let distance = delta.length();
                let blocked = colliders
                    .iter()
                    .filter_map(|(c, t, member, name, identity)| {
                        // Robot root support cylinders enclose armor and are explicitly excluded from bullet geometry.
                        // Use the combat inner core/armor/environment colliders for this approximation.
                        if identity.is_some() || member.is_some_and(|m| m.id == CONTROLLED_ROBOT_ID)
                        {
                            return None;
                        }
                        c.cast_ray(
                            t.translation(),
                            t.rotation(),
                            camera_pos,
                            delta / distance,
                            (distance - 0.01).max(0.0),
                            false,
                        )
                        .map(|hit| (hit.0, name.map(|n| n.as_str()), member.map(|m| m.id.0)))
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0));
                if let Some((hit, name, owner)) = blocked {
                    obstruction = json!({"distance_m":hit,"point_distance_m":distance,"collider_name":name,"owner":owner});
                    reason = "occluded";
                    break;
                }
            }
        }
        evaluation.push(
            json!({"center_world":center.to_array(),"corners_world":plate.corners_world,
            "ideal_corners_px":pixels,"visibility":reason,"occlusion":obstruction}),
        );
        if reason == "visible" {
            detections.push(
                json!({"label":plate.label,"color":plate.team,"objectness":1.0,"corners":pixels}),
            );
        }
    }
    Ok((
        json!({"round_id":round.id,"capture_time_ns":clock.elapsed.as_nanos() as u64,
        "capture_timestamp_ns":clock.timestamp_ns(),
        "camera":{"width":k.width,"height":k.height,"fx":k.fx,"fy":k.fy,"cx":k.cx,"cy":k.cy,"distortion":[0,0,0,0,0]},
        "kinematics":{"world_t_gimbal":transform_data(world_gimbal),"gimbal_t_camera_optical":transform_data(relative_transform(world_gimbal,world_camera)),
            "gimbal_t_muzzle":transform_data(local_muzzle)},
        "feedback":{"valid":g.valid,"timestamp_ns":g.state_timestamp_ns,"yaw_rad":g.actual_yaw_rad,"pitch_rad":g.actual_pitch_rad,
            "yaw_velocity_rad_s":g.yaw_velocity_rad_s,"pitch_velocity_rad_s":g.pitch_velocity_rad_s,
            "consumed_command_timestamp_ns":g.consumed_command_timestamp_ns,
"mode":g.mode,"consumed_at_timestamp_ns":g.consumed_at_timestamp_ns,
"target_yaw_rad":g.target_yaw_rad,"target_pitch_rad":g.target_pitch_rad,
"yaw_acceleration_rad_s2":g.yaw_acceleration_rad_s2,"pitch_acceleration_rad_s2":g.pitch_acceleration_rad_s2,"command_valid":g.command_valid,"saturation_flags":g.saturation_flags},
        "detections":detections}),
        evaluation,
    ))
}

/// Admission uses the noise-free optical geometry, even when measurements are disabled.
/// It does not consume a detector RNG value, create a frame, or advance either clock.
pub(super) fn validate_initial_visibility(app: &mut App) -> Result<Value, String> {
    let (frame, plates) = app
        .world_mut()
        .run_system_once(capture)
        .map_err(|e| e.to_string())??;
    let count = frame["detections"]
        .as_array()
        .ok_or("missing initial detections")?
        .len();
    if count == 0 {
        let reasons: Vec<_> = plates.iter().map(|p| p["visibility"].clone()).collect();
        return Err(format!(
            "no complete visible blue armor in red camera: {}",
            json!(reasons)
        ));
    }
    Ok(json!({"visible_armor_count":count, "plates":plates,
        "camera":frame["camera"], "kinematics":frame["kinematics"],
        "criterion":"front_facing_all_corners_in_frame_center_and_corners_unoccluded"}))
}

pub(super) fn tick(app: &mut App) -> Result<(), String> {
    if !app.world().contains_resource::<Measurements>() {
        return Ok(());
    }
    let now = app.world().resource::<Time<Fixed>>().elapsed().as_nanos() as u64;
    let index = app.world().resource::<Measurements>().next_index;
    if now as u128 * 30 >= index as u128 * 1_000_000_000 {
        let (mut frame, truth) = app
            .world_mut()
            .run_system_once(capture)
            .map_err(|e| e.to_string())??;
        let mut m = app.world_mut().resource_mut::<Measurements>();
        let delivery = now + m.config.latency_ms * 1_000_000;
        frame["sequence"] = json!(index);
        frame["delivery_time_ns"] = json!(delivery);
        let dropout = m.rng.random::<f64>() < m.config.dropout_probability
            || m.config
                .blackouts_ms
                .iter()
                .any(|p| now / 1_000_000 >= p[0] && now / 1_000_000 < p[1]);
        if dropout {
            frame["detections"] = json!([]);
        } else {
            let std = m.config.noise_std_px;
            for d in frame["detections"].as_array_mut().unwrap() {
                for p in d["corners"].as_array_mut().unwrap() {
                    for xy in p.as_array_mut().unwrap() {
                        let u = m.rng.random::<f64>().max(f64::MIN_POSITIVE);
                        let v = m.rng.random::<f64>();
                        *xy = json!(
                            xy.as_f64().unwrap()
                                + std * (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
                        );
                    }
                }
            }
        }
        let evaluation = json!({"sequence":index,"capture_time_ns":now,"plates":truth,"detector_dropout":dropout});
        m.pending.push_back((delivery, frame, evaluation));
        m.next_index += 1;
    }
    let mut m = app.world_mut().resource_mut::<Measurements>();
    while m.pending.front().is_some_and(|p| p.0 <= now) {
        let (_, frame, truth) = m.pending.pop_front().unwrap();
        m.frames.push(frame);
        m.truth.push(truth);
    }
    if m.frames.len() > 32 {
        return Err("undelivered visual frame overflow".into());
    }
    Ok(())
}

pub(super) fn attach(w: &World, data: &mut Value) {
    if let Some(m) = w.get_resource::<Measurements>() {
        data["visual_frames"] = json!(m.frames);
        data["evaluation"]["visual_truth"] = json!(m.truth);
    }
}
pub(super) fn commit(w: &mut World) {
    if let Some(mut m) = w.get_resource_mut::<Measurements>() {
        m.frames.clear();
        m.truth.clear();
    }
}
