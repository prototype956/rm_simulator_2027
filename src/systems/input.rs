use bevy::prelude::*;
use std::sync::atomic::Ordering;

use crate::components::{
    ActiveSlapper, Controlled, Infantry, InfantryChassis, InfantryGimbal, SlapperInfantry,
    SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::vehicle::movement::VehicleDynamic;
use crate::systems::ControllerState;
use avian3d::prelude::*;

const CHASSIS_ROTATION_RESPONSE: f32 = 40.0;
const CHASSIS_ROTATION_STOP_EPSILON: f32 = 1e-3;
const CHASSIS_TILT_LIMIT: f32 = 20.0 * std::f32::consts::PI / 180.0;

fn update_chassis_rotation(
    chassis_transform: &mut Transform,
    chassis_data: &mut InfantryChassis,
    yaw_input: f32,
    roll_input: f32,
    pitch_input: f32,
    yaw_rotation_speed: f32,
    tilt_rotation_speed: f32,
    dt: f32,
) {
    let target_yaw_velocity = yaw_input * yaw_rotation_speed;
    let response = 1.0 - (-CHASSIS_ROTATION_RESPONSE * dt).exp();
    chassis_data.yaw_velocity += (target_yaw_velocity - chassis_data.yaw_velocity) * response;

    if chassis_data.yaw_velocity.abs() < CHASSIS_ROTATION_STOP_EPSILON
        && target_yaw_velocity.abs() < CHASSIS_ROTATION_STOP_EPSILON
    {
        chassis_data.yaw_velocity = 0.0;
    }

    chassis_data.yaw += chassis_data.yaw_velocity * dt;
    chassis_data.roll = (chassis_data.roll + roll_input * tilt_rotation_speed * dt)
        .clamp(-CHASSIS_TILT_LIMIT, CHASSIS_TILT_LIMIT);
    chassis_data.pitch = (chassis_data.pitch + pitch_input * tilt_rotation_speed * dt)
        .clamp(-CHASSIS_TILT_LIMIT, CHASSIS_TILT_LIMIT);
    chassis_transform.rotation = Quat::from_euler(
        EulerRot::YXZ,
        chassis_data.yaw,
        chassis_data.pitch,
        chassis_data.roll,
    );
}

pub fn vehicle_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    config: Res<SimulationConfig>,
    infantry: Single<(Forces, &Mass, &mut VehicleDynamic), (With<Infantry>, With<Controlled>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<Controlled>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (
            With<Controlled>,
            Without<InfantryGimbal>,
            With<InfantryChassis>,
            Without<Infantry>,
        ),
    >,
) {
    let controller = controller.controlled;
    let input = controller.movement;
    let boost = controller.boost_multiplier();

    let (mut forces, &Mass(mass), mut dynamic) = infantry.into_inner();

    let dt = time.delta_secs();
    dynamic.linear(
        &mut forces,
        mass,
        gimbal.into_inner().0,
        input,
        time.delta_secs(),
        boost,
    );

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    update_chassis_rotation(
        &mut chassis_transform,
        &mut chassis_data,
        controller.chassis_yaw,
        controller.chassis_roll,
        controller.chassis_pitch,
        config.vehicle.rotation_speed,
        config.vehicle.tilt_rotation_speed,
        dt,
    );
}

pub fn remote_vehicle_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    config: Res<SimulationConfig>,
    infantry: Single<
        (&GlobalTransform, Forces, &Mass, &mut VehicleDynamic),
        (With<ActiveSlapper>, With<Infantry>, Without<Controlled>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (
            With<ActiveSlapper>,
            With<InfantryChassis>,
            Without<InfantryGimbal>,
            Without<Infantry>,
        ),
    >,
) {
    let controller = controller.remote;
    let input = controller.movement;
    let boost = controller.boost_multiplier();

    let (infantry_global_transform, mut forces, &Mass(mass), mut dynamic) = infantry.into_inner();

    let dt = time.delta_secs();
    dynamic.linear(
        &mut forces,
        mass,
        infantry_global_transform,
        input,
        time.delta_secs(),
        boost,
    );

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    update_chassis_rotation(
        &mut chassis_transform,
        &mut chassis_data,
        controller.chassis_yaw,
        controller.chassis_roll,
        controller.chassis_pitch,
        config.vehicle.rotation_speed,
        config.vehicle.tilt_rotation_speed,
        dt,
    );
}

pub fn gimbal_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    enabled: Res<SubscribeAutoAim>,
    config: Res<SimulationConfig>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<Controlled>, Without<InfantryChassis>),
    >,
) {
    if enabled.load(Ordering::Acquire) {
        return;
    }

    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    let controller = controller.controlled;
    let rotation_speed = config.vehicle.gimbal_rotation_speed * controller.gimbal_scale() * dt;
    gimbal_data.local_yaw += controller.gimbal.x * rotation_speed;
    gimbal_data.pitch += controller.gimbal.y * rotation_speed;

    gimbal_data.pitch = gimbal_data.pitch.clamp(
        -config.vehicle.gimbal_pitch_limit,
        config.vehicle.gimbal_pitch_limit,
    );

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

pub fn remote_gimbal_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    config: Res<SimulationConfig>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<ActiveSlapper>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    let controller = controller.remote;
    let rotation_speed = config.vehicle.gimbal_rotation_speed * controller.gimbal_scale() * dt;
    gimbal_data.local_yaw += controller.gimbal.x * rotation_speed;
    gimbal_data.pitch += controller.gimbal.y * rotation_speed;
    gimbal_data.pitch = gimbal_data.pitch.clamp(
        -config.vehicle.gimbal_pitch_limit,
        config.vehicle.gimbal_pitch_limit,
    );

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

pub fn switch_slapper_control(
    mut commands: Commands,
    controller: Res<ControllerState>,
    children: Query<&Children>,
    slapper_roots: Query<Entity, (With<Infantry>, With<SlapperInfantry>)>,
    active_root: Query<Entity, (With<Infantry>, With<SlapperInfantry>, With<ActiveSlapper>)>,
) {
    if !controller.controlled.switch_slapper_just_pressed {
        return;
    }

    let roots: Vec<Entity> = slapper_roots.iter().collect();
    if roots.len() <= 1 {
        return;
    }

    let current = active_root.single().ok();
    let current_idx = current.and_then(|e| roots.iter().position(|&r| r == e));
    let next_idx = match current_idx {
        Some(idx) => (idx + 1) % roots.len(),
        None => 0,
    };

    // Remove ActiveSlapper from current
    if let Some(current_root) = current {
        commands.entity(current_root).remove::<ActiveSlapper>();
        for descendant in children.iter_descendants(current_root) {
            commands.entity(descendant).remove::<ActiveSlapper>();
        }
    }

    // Add ActiveSlapper to next
    let next_root = roots[next_idx];
    commands.entity(next_root).insert(ActiveSlapper);
    for descendant in children.iter_descendants(next_root) {
        commands.entity(descendant).insert(ActiveSlapper);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chassis_rotation_smoothly_ramps_towards_target_speed() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis::default();

        update_chassis_rotation(
            &mut transform,
            &mut chassis,
            1.0,
            0.0,
            0.0,
            9.42,
            2.0,
            0.016,
        );

        assert!(chassis.yaw_velocity > 0.0);
        assert!(chassis.yaw_velocity < 9.42);
        assert!(chassis.yaw > 0.0);
    }

    #[test]
    fn chassis_rotation_uses_independent_yaw_and_tilt_speeds() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis::default();

        update_chassis_rotation(&mut transform, &mut chassis, 1.0, 1.0, -1.0, 8.0, 0.25, 1.0);

        assert!(chassis.yaw_velocity > 0.25);
        assert_eq!(chassis.roll, 0.25);
        assert_eq!(chassis.pitch, -0.25);
    }

    #[test]
    fn chassis_rotation_smoothly_brakes_to_stop() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis {
            yaw: 0.0,
            yaw_velocity: 9.42,
            ..default()
        };

        for _ in 0..60 {
            update_chassis_rotation(
                &mut transform,
                &mut chassis,
                0.0,
                0.0,
                0.0,
                9.42,
                2.0,
                0.016,
            );
        }

        assert!(chassis.yaw_velocity.abs() < 1e-2);
    }

    #[test]
    fn chassis_rotation_bounds_roll_and_pitch_as_swing_angles() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis::default();

        update_chassis_rotation(&mut transform, &mut chassis, 0.0, 1.0, -1.0, 2.0, 2.0, 10.0);
        assert_eq!(chassis.roll, CHASSIS_TILT_LIMIT);
        assert_eq!(chassis.pitch, -CHASSIS_TILT_LIMIT);

        update_chassis_rotation(&mut transform, &mut chassis, 1.0, -1.0, 1.0, 2.0, 2.0, 10.0);

        assert_eq!(chassis.roll, -CHASSIS_TILT_LIMIT);
        assert_eq!(chassis.pitch, CHASSIS_TILT_LIMIT);
        let (_, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
        assert!((roll + CHASSIS_TILT_LIMIT).abs() < 1e-5);
        assert!((pitch - CHASSIS_TILT_LIMIT).abs() < 1e-5);
    }
}
