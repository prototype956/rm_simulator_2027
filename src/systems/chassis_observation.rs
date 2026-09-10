use avian3d::prelude::{AngularVelocity, LinearVelocity};
use bevy::prelude::*;

use crate::components::{Controlled, Infantry};
use crate::config::{MecanumConfig, SimulationConfig};

const NUM_WHEELS: usize = 4;
const MIN_RADIUS_M: f32 = 1e-6;

#[derive(Resource, Debug, Clone)]
pub struct ChassisObservationFrame {
    pub stamp_s: f64,
    pub dt_s: f32,
    pub v_body: Vec2,
    pub wz_radps: f32,
    // Wheel order: [FL, FR, RL, RR]
    pub wheel_linear_mps: [f32; NUM_WHEELS],
    // Wheel order: [FL, FR, RL, RR]
    pub wheel_angular_radps: [f32; NUM_WHEELS],
    pub a_body: Vec2,
    pub alpha_z_radps2: f32,
    pub rpy_rad: Vec3,
    pub gyro_xyz_radps: Vec3,
    pub accel_xyz_mps2: Vec3,
}

impl Default for ChassisObservationFrame {
    fn default() -> Self {
        Self {
            stamp_s: 0.0,
            dt_s: 0.0,
            v_body: Vec2::ZERO,
            wz_radps: 0.0,
            wheel_linear_mps: [0.0; NUM_WHEELS],
            wheel_angular_radps: [0.0; NUM_WHEELS],
            a_body: Vec2::ZERO,
            alpha_z_radps2: 0.0,
            rpy_rad: Vec3::ZERO,
            gyro_xyz_radps: Vec3::ZERO,
            accel_xyz_mps2: Vec3::ZERO,
        }
    }
}

#[derive(Resource, Debug, Clone, Default)]
pub struct PreviousKinematicState {
    initialized: bool,
    v_body: Vec2,
    wz_radps: f32,
}

pub fn update_chassis_observation(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut frame: ResMut<ChassisObservationFrame>,
    mut previous: ResMut<PreviousKinematicState>,
    chassis: Query<
        (&GlobalTransform, &LinearVelocity, &AngularVelocity),
        (With<Infantry>, With<Controlled>),
    >,
) {
    let Ok((chassis_tf, linear_velocity, angular_velocity)) = chassis.single() else {
        *frame = ChassisObservationFrame::default();
        *previous = PreviousKinematicState::default();
        return;
    };

    let stamp_s = time.elapsed_secs_f64();
    let dt_s = time.delta_secs();
    let rotation = chassis_tf.compute_transform().rotation;

    // Convert from world velocity to chassis-local velocity, then remap Bevy axes
    // (right, up, back) to body axes (forward, left, up).
    let linear_local_bevy = rotation.inverse() * linear_velocity.0;
    let linear_body = bevy_local_to_body(linear_local_bevy);
    let v_body = Vec2::new(linear_body.x, linear_body.y);

    let angular_local_bevy = rotation.inverse() * angular_velocity.0;
    let gyro_body = bevy_local_to_body(angular_local_bevy);
    let wz_radps = gyro_body.z;

    let (a_body, alpha_z_radps2) = compute_body_acceleration(&previous, v_body, wz_radps, dt_s);

    let body_rotation = bevy_to_body_quat(rotation);
    let (roll, pitch, yaw) = body_rotation.to_euler(EulerRot::XYZ);

    let wheel_linear_mps = mecanum_wheel_linear(v_body.x, v_body.y, wz_radps, &config.mecanum);
    let wheel_angular_radps =
        wheel_linear_to_angular(wheel_linear_mps, config.mecanum.wheel_radius_m);

    *frame = ChassisObservationFrame {
        stamp_s,
        dt_s,
        v_body,
        wz_radps,
        wheel_linear_mps,
        wheel_angular_radps,
        a_body,
        alpha_z_radps2,
        rpy_rad: Vec3::new(roll, pitch, yaw),
        gyro_xyz_radps: gyro_body,
        accel_xyz_mps2: Vec3::new(a_body.x, a_body.y, 0.0),
    };

    previous.initialized = true;
    previous.v_body = v_body;
    previous.wz_radps = wz_radps;
}

fn compute_body_acceleration(
    previous: &PreviousKinematicState,
    v_body: Vec2,
    wz_radps: f32,
    dt_s: f32,
) -> (Vec2, f32) {
    if !previous.initialized || dt_s <= f32::EPSILON {
        return (Vec2::ZERO, 0.0);
    }

    let inv_dt = 1.0 / dt_s;
    (
        (v_body - previous.v_body) * inv_dt,
        (wz_radps - previous.wz_radps) * inv_dt,
    )
}

fn mecanum_wheel_linear(vx: f32, vy: f32, wz: f32, config: &MecanumConfig) -> [f32; NUM_WHEELS] {
    let k = config.half_wheelbase_m + config.half_trackwidth_m;
    [
        vx - vy - k * wz,
        vx + vy + k * wz,
        vx + vy - k * wz,
        vx - vy + k * wz,
    ]
}

fn wheel_linear_to_angular(
    wheel_linear_mps: [f32; NUM_WHEELS],
    wheel_radius_m: f32,
) -> [f32; NUM_WHEELS] {
    let radius = wheel_radius_m.max(MIN_RADIUS_M);
    wheel_linear_mps.map(|wheel_linear| wheel_linear / radius)
}

fn bevy_local_to_body(vector: Vec3) -> Vec3 {
    Vec3::new(-vector.z, -vector.x, vector.y)
}

fn bevy_to_body_quat(rotation: Quat) -> Quat {
    let align = Quat::from_mat3(&Mat3::from_cols(
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, 0.0),
    ));
    align * rotation * align.inverse()
}
