use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::config::SimulationConfig;
use crate::talos::capture::{
    TalosCaptureContext, TalosCapturePlugin, TalosFrameStamp, advance_talos_frame_stamp,
    publish_talos_runtime_state_system,
};
use crate::talos::gimbal_actuator::{
    GimbalActuator, GimbalActuatorTelemetry, spawn_command_receiver, update_gimbal_actuator,
};
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use talos_ipc::*;

#[derive(Resource, Deref, DerefMut)]
pub struct TalosEnabled(pub AtomicBool);

pub struct TalosPluginConfig {
    pub width: u32,
    pub height: u32,
    pub fov_y: f32,
    pub texture_format: TextureFormat,
}

impl Default for TalosPluginConfig {
    fn default() -> Self {
        let config = SimulationConfig::default();
        Self {
            width: config.capture.color.width,
            height: config.capture.color.height,
            fov_y: config.camera.fov.to_radians(),
            texture_format: TextureFormat::Rgba8UnormSrgb,
        }
    }
}

#[derive(Default)]
pub struct TalosPlugin {
    pub config: TalosPluginConfig,
}

impl Plugin for TalosPlugin {
    fn build(&self, app: &mut App) {
        let publisher = match ShmPublisher::create_with_image(
            self.config.width,
            self.config.height,
            talos_ipc::ImageFormat::Bgr8,
        ) {
            Ok(p) => {
                info!("talos shm created");
                p
            }
            Err(e) => {
                error!("cannot create talos shm: {}", e);
                return;
            }
        };

        let publisher = Arc::new(Mutex::new(publisher));

        let capture_config = CaptureConfig {
            width: self.config.width,
            height: self.config.height,
            texture_format: self.config.texture_format,
            frame_kind: CapturedFrameKind::Bgr8,
        };

        let capture_context = TalosCaptureContext {
            publisher: publisher.clone(),
            fov_y: self.config.fov_y,
        };

        app.init_resource::<TalosFrameStamp>();

        app.add_plugins(TalosCapturePlugin {
            config: capture_config,
            context: capture_context,
        });

        app.init_resource::<GimbalActuator>()
            .init_resource::<GimbalActuatorTelemetry>();
        let receiver_connected = match ShmSubscriber::connect() {
            Ok(subscriber) => {
                info!("connected to talos-cpp");
                let config = SimulationConfig::default();
                let (inbox, worker) =
                    spawn_command_receiver(subscriber, config.gimbal_actuator.command_poll_hz);
                app.insert_resource(inbox)
                    .insert_resource(worker)
                    .add_systems(
                        PostUpdate,
                        update_gimbal_actuator.before(TransformSystems::Propagate),
                    );
                true
            }
            Err(_) => {
                info!("could not connect to talos-cpp");
                false
            }
        };

        app.insert_resource(TalosEnabled(AtomicBool::new(true)));
        app.add_systems(Last, (advance_talos_frame_stamp, heartbeat_system));
        if receiver_connected {
            app.add_systems(
                Last,
                publish_talos_runtime_state_system.after(advance_talos_frame_stamp),
            );
        } else {
            app.add_systems(
                Last,
                publish_talos_runtime_state_system.after(advance_talos_frame_stamp),
            );
        }
    }
}

fn heartbeat_system(context: Option<Res<TalosCaptureContext>>) {
    if let Some(ctx) = context {
        if let Ok(mut publisher) = ctx.publisher.lock() {
            publisher.update_heartbeat();
        }
    }
}

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn to_ros(bevy_transform: Transform) -> Transform {
    let new_rotation = to_ros_quat(bevy_transform.rotation);
    let new_translation = to_ros_translation(bevy_transform.translation);
    Transform::from_translation(new_translation).with_rotation(new_rotation)
}

pub fn to_ros_translation(vec3: Vec3) -> Vec3 {
    let align_rot_mat = M_ALIGN_MAT3;
    let new_translation = align_rot_mat * vec3;
    new_translation
}

pub fn to_ros_quat(quat: Quat) -> Quat {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * quat * align_quat.inverse();
    new_rotation
}
