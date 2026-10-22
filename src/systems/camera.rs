use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use std::f32::consts::PI;

use crate::components::{
    CameraMode, Controlled, FollowingType, Infantry, InfantryLaunchOffset, InfantryViewOffset,
    MainCamera,
};
use crate::config::SimulationConfig;
use crate::systems::ControllerState;

pub fn following_controls(mut mode: ResMut<CameraMode>, controller: Res<ControllerState>) {
    if controller.controlled.switch_camera_just_pressed {
        mode.0 = match mode.0 {
            FollowingType::Free => FollowingType::Robot,
            FollowingType::Robot => FollowingType::ThirdPerson,
            FollowingType::ThirdPerson => FollowingType::Free,
        };
    }
}

/// Camera pose in Bevy world coordinates for the vehicle GLB's local camera/muzzle offsets.
/// Shared by rendered views and synthetic projection so spawn visibility uses the same optics.
pub fn robot_camera_pose(
    root: &Transform,
    gimbal: &Transform,
    view: &Transform,
    muzzle: &Transform,
) -> Transform {
    let rotation = root.rotation * gimbal.rotation;
    Transform::from_translation(root.translation + rotation * view.translation)
        .with_rotation(rotation * muzzle.rotation * Quat::from_rotation_x(PI / 2.0))
}

/// Resolve the actual mount hierarchy, including body lift, fold mount and local scale.
pub fn robot_camera_from_mounts(view: GlobalTransform, muzzle: GlobalTransform) -> Transform {
    Transform::from_translation(view.translation())
        .with_rotation(muzzle.rotation() * Quat::from_rotation_x(PI / 2.0))
}

pub fn update_camera_follow(
    camera_query: Single<(Entity, &MainCamera), Without<Controlled>>,
    infantry: Single<&Transform, (With<Infantry>, With<Controlled>)>,
    view_offset: Single<Entity, (With<Controlled>, With<InfantryViewOffset>)>,
    launch_offset: Single<Entity, (With<Controlled>, With<InfantryLaunchOffset>)>,
    mut transforms: ParamSet<(
        TransformHelper,
        Query<&mut Transform, (With<MainCamera>, Without<Infantry>)>,
    )>,
    mode: Res<CameraMode>,
) {
    let (camera_entity, camera_offset) = camera_query.into_inner();
    let mut desired = None;
    match mode.0 {
        FollowingType::Robot => {
            let helper = transforms.p0();
            if let (Ok(view), Ok(muzzle)) = (
                helper.compute_global_transform(*view_offset),
                helper.compute_global_transform(*launch_offset),
            ) {
                desired = Some(robot_camera_from_mounts(view, muzzle));
            }
        }
        FollowingType::ThirdPerson => {
            let base_transform = infantry.into_inner();
            let offset = base_transform.rotation * camera_offset.follow_offset;
            desired = Some(
                Transform::from_translation(base_transform.translation + offset)
                    .looking_at(base_transform.translation, Vec3::Y),
            );
        }
        FollowingType::Free => {}
    }
    if let Some(pose) = desired {
        if let Ok(mut transform) = transforms.p1().get_mut(camera_entity) {
            *transform = pose;
        }
    }
}

pub fn freecam_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    config: Res<SimulationConfig>,
    mut mouse_motion_events: MessageReader<MouseMotion>,
    keyboard: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&mut Transform, (With<MainCamera>, Without<Infantry>)>,
) {
    if mode.0 != FollowingType::Free {
        return;
    }

    let delta = time.delta_secs();
    let mut camera_transform = camera_query.into_inner();

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    if mouse_delta != Vec2::ZERO {
        let (yaw, pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let new_yaw = yaw - mouse_delta.x * config.camera.mouse_sensitivity;
        let new_pitch = (pitch - mouse_delta.y * config.camera.mouse_sensitivity).clamp(-1.4, 1.4);

        camera_transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, roll);
    }

    let speed = config.camera.free_move_speed * delta;
    let forward = camera_transform.forward();
    let right = camera_transform.right();
    let up = camera_transform.up();

    if keyboard.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        camera_transform.translation -= forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        camera_transform.translation -= right * speed;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * speed;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        camera_transform.translation += up * speed;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        camera_transform.translation -= up * speed;
    }
}
