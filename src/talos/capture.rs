use crate::capture::{
    CameraFov, CaptureBundle, CaptureSource, ImageHandle, compute_camera_intrinsics,
    driver::{
        CaptureConfig, CaptureFrameId, CapturedFrame, CapturedFrameKind, GpuCaptureHandler,
        SnapshotAsync, SnapshotSync,
    },
    setup_capture_camera, setup_preview_window, sync_capture_camera,
};
use crate::capture_geometry::{
    capture_ground_truth, compose_transform, quat_from_wire, relative_transform,
    transform_from_axes, transform_near,
};
use crate::components::{
    Controlled, Infantry, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::robomaster::combat::RobotIdentity;
use crate::robomaster::combat::reset::{RoundFence, TrainingRound};
use crate::robomaster::prelude::{ArmorParts, ArmorRoot, VertexData};
use crate::statistic::ProjectileStatistics;
use crate::systems::{ChassisObservationFrame, GameplaySystems};
use crate::talos::gimbal_actuator::GimbalActuatorTelemetry;
use crate::talos::plugin::to_ros_translation;
use bevy::ecs::world::DeferredWorld;
use bevy::prelude::*;
use bevy::render::{Extract, ExtractSchedule, RenderApp, RenderSystems};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::*;

static FRAME_SEQ: AtomicU64 = AtomicU64::new(0);

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct TalosFrameStamp {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
}

pub fn advance_talos_frame_stamp(mut stamp: ResMut<TalosFrameStamp>) {
    stamp.frame_seq = FRAME_SEQ.fetch_add(1, Ordering::Relaxed);
    stamp.timestamp_ns = now_ns();
}

/// Extracted pose data from MainApp to RenderApp for synchronized publishing
#[derive(Resource, Clone, Default)]
pub struct ExtractedPoseData {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pose: Option<CapturedPoseData>,
    pub valid: bool,
    round_id: u64,
    fence: Option<RoundFence>,
}

/// Pose data captured at frame snapshot time
#[derive(Clone)]
struct CapturedPoseData {
    camera_info: CameraInfo,
    world_t_gimbal: RigidTransformF32,
    gimbal_t_camera_optical: RigidTransformF32,
    gimbal_t_muzzle: RigidTransformF32,
    actuator: GimbalActuatorTelemetry,
    projectile_statistics: ProjectileStatisticsMeta,
    chassis_observation: ChassisObservation,
    ground_truth: GroundTruthBatch,
    combat: CombatFrameMeta,
}

#[derive(Resource, Debug, Clone, Copy)]
struct TalosCameraCalibration(CameraInfo);

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

struct TalosSnapshotSync {
    round_id: u64,
    fence: RoundFence,
    frame_seq: u64,
    timestamp_ns: u64,
    pose: CapturedPoseData,
}

impl SnapshotSync for TalosSnapshotSync {
    fn captured(
        self: Box<Self>,
        world: &mut DeferredWorld,
        config: &CaptureConfig,
    ) -> Box<dyn SnapshotAsync> {
        let ctx = world.resource::<TalosCaptureContextShared>().0.clone();

        Box::new(TalosSnapshot {
            round_id: self.round_id,
            fence: self.fence,
            ctx,
            frame_seq: self.frame_seq,
            timestamp_ns: self.timestamp_ns,
            pose: self.pose,
            expected_width: config.width,
            expected_height: config.height,
        })
    }
}

struct TalosSnapshot {
    round_id: u64,
    fence: RoundFence,
    ctx: Arc<Mutex<ShmPublisher>>,
    frame_seq: u64,
    timestamp_ns: u64,
    pose: CapturedPoseData,
    expected_width: u32,
    expected_height: u32,
}

impl SnapshotAsync for TalosSnapshot {
    fn captured(&mut self, frame: CapturedFrame<'_>) {
        if frame.kind != CapturedFrameKind::Bgr8 {
            return;
        }

        let expected_size = (frame.width * frame.height * 3) as usize;
        if frame.data.len() != expected_size {
            warn!(
                "图像大小不匹配: expected {} bytes, got {} bytes",
                expected_size,
                frame.data.len()
            );
            return;
        }

        if frame.width != self.expected_width || frame.height != self.expected_height {
            warn!(
                "image resolution mismatched: expected {}x{}, got {}x{}",
                self.expected_width, self.expected_height, frame.width, frame.height
            );
            return;
        }

        let Some(_generation) = self.fence.lock_round(self.round_id) else {
            debug!(
                "discard old capture round={} frame={}",
                self.round_id, self.frame_seq
            );
            return;
        };
        if let Ok(mut publisher) = self.ctx.lock() {
            let mut camera_info = self.pose.camera_info;
            camera_info.timestamp_ns = self.timestamp_ns;
            let metadata = CapturedFrameMeta {
                frame_seq: self.frame_seq,
                capture_timestamp_ns: self.timestamp_ns,
                gimbal_consumed_command_timestamp_ns: self
                    .pose
                    .actuator
                    .consumed_command_timestamp_ns,
                gimbal_yaw_velocity_rad_s: self.pose.actuator.yaw_velocity_rad_s,
                gimbal_pitch_velocity_rad_s: self.pose.actuator.pitch_velocity_rad_s,
                gimbal_yaw_acceleration_rad_s2: self.pose.actuator.yaw_acceleration_rad_s2,
                gimbal_pitch_acceleration_rad_s2: self.pose.actuator.pitch_acceleration_rad_s2,
                gimbal_actuator_mode: self.pose.actuator.mode,
                gimbal_saturation_flags: self.pose.actuator.saturation_flags,
                gimbal_telemetry_valid: u8::from(self.pose.actuator.valid),
                gimbal_command_valid: u8::from(self.pose.actuator.command_valid),
                camera_info,
                world_t_gimbal: self.pose.world_t_gimbal,
                gimbal_t_camera_optical: self.pose.gimbal_t_camera_optical,
                gimbal_t_muzzle: self.pose.gimbal_t_muzzle,
                projectile_statistics: self.pose.projectile_statistics,
                chassis_observation: self.pose.chassis_observation,
                ground_truth: self.pose.ground_truth,
                combat: self.pose.combat,
                ..default()
            };
            if publisher.try_publish_frame(frame.data, metadata) {
                debug!(
                    "round frame round={} frame={} timestamp_ns={}",
                    self.round_id, self.frame_seq, self.timestamp_ns
                );
            }
        }
    }
}

#[derive(Default)]
struct TalosSnapshotCreator {}

impl GpuCaptureHandler for TalosSnapshotCreator {
    fn captured(
        &self,
        world: &World,
        _frame_id: Option<CaptureFrameId>,
    ) -> Option<Box<dyn SnapshotSync>> {
        // Timestamp, frame sequence and pose must come from the same ExtractSchedule snapshot.
        let extracted = world.get_resource::<ExtractedPoseData>()?;
        if !extracted.valid {
            return None;
        }
        let pose = extracted.pose.clone()?;

        Some(Box::new(TalosSnapshotSync {
            round_id: extracted.round_id,
            fence: extracted.fence.clone()?,
            frame_seq: extracted.frame_seq,
            timestamp_ns: extracted.timestamp_ns,
            pose,
        }))
    }
}

#[derive(Resource, Clone, Deref, DerefMut)]
pub struct TalosCaptureContextShared(pub Arc<Mutex<ShmPublisher>>);

#[derive(Resource, Clone)]
pub struct TalosCaptureContext {
    pub publisher: Arc<Mutex<ShmPublisher>>,
    pub fov_y: f32,
}

pub struct TalosCapturePlugin {
    pub config: CaptureConfig,
    pub context: TalosCaptureContext,
}

pub fn publish_talos_runtime_state_system(
    context: Option<Res<TalosCaptureContext>>,
    frame_stamp: Res<TalosFrameStamp>,
    following: Res<SubscribeAutoAim>,
    actuator: Res<GimbalActuatorTelemetry>,
) {
    let Some(ctx) = context else {
        return;
    };

    if let Ok(mut publisher) = ctx.publisher.lock() {
        publisher.publish_runtime_state(RuntimeState {
            timestamp_ns: frame_stamp.timestamp_ns,
            consumed_command_timestamp_ns: actuator.consumed_command_timestamp_ns,
            consumed_at_timestamp_ns: actuator.consumed_at_timestamp_ns,
            target_yaw_rad: actuator.target_yaw_rad,
            target_pitch_rad: actuator.target_pitch_rad,
            actual_yaw_rad: actuator.actual_yaw_rad,
            actual_pitch_rad: actuator.actual_pitch_rad,
            yaw_velocity_rad_s: actuator.yaw_velocity_rad_s,
            pitch_velocity_rad_s: actuator.pitch_velocity_rad_s,
            yaw_acceleration_rad_s2: actuator.yaw_acceleration_rad_s2,
            pitch_acceleration_rad_s2: actuator.pitch_acceleration_rad_s2,
            following: u8::from(following.load(Ordering::Acquire)),
            actuator_mode: actuator.mode,
            saturation_flags: actuator.saturation_flags,
            command_valid: u8::from(actuator.command_valid),
            _pad: [0; 4],
        });
    }
}

impl Plugin for TalosCapturePlugin {
    fn build(&self, app: &mut App) {
        let capture = CaptureBundle::color(
            app,
            self.config.clone(),
            vec![Box::new(TalosSnapshotCreator::default())],
        );
        let render_target_handle = capture.color_target().unwrap().clone();

        let intrinsics =
            compute_camera_intrinsics(self.config.width, self.config.height, self.context.fov_y);
        let camera_info = CameraInfo {
            timestamp_ns: 0,
            fx: intrinsics.fx,
            fy: intrinsics.fy,
            cx: intrinsics.cx,
            cy: intrinsics.cy,
            distortion: [0.0; 5],
            width: intrinsics.width,
            height: intrinsics.height,
            _pad: [0; 24],
        };

        app.add_plugins(capture)
            .insert_resource(ImageHandle(render_target_handle))
            .insert_resource(CameraFov(self.context.fov_y))
            .insert_resource(TalosCameraCalibration(camera_info))
            .insert_resource(self.context.clone())
            .add_systems(Startup, setup_capture_camera)
            .add_systems(Startup, setup_preview_window)
            .add_systems(
                Update,
                sync_capture_camera
                    .after(GameplaySystems::Camera)
                    .before(RenderSystems::Render),
            );

        app.sub_app_mut(RenderApp)
            .insert_resource(TalosCaptureContextShared(self.context.publisher.clone()))
            .insert_resource(self.context.clone())
            .insert_resource(ExtractedPoseData::default())
            .add_systems(ExtractSchedule, extract_pose_data);
    }
}

/// Extract pose data from MainApp to RenderApp
fn extract_pose_data(
    mut pose_data: ResMut<ExtractedPoseData>,
    stamps: (
        Extract<Res<TalosFrameStamp>>,
        Extract<Res<TrainingRound>>,
        Extract<Res<RoundFence>>,
    ),
    camera: Extract<Query<&GlobalTransform, With<CaptureSource>>>,
    gimbal: Extract<Query<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>>,
    muzzle_offset: Extract<
        Query<(&GlobalTransform, &Transform), (With<InfantryLaunchOffset>, With<Controlled>)>,
    >,
    chassis_obs: Extract<Res<ChassisObservationFrame>>,
    telemetry: (
        Extract<Res<GimbalActuatorTelemetry>>,
        Extract<Res<ProjectileStatistics>>,
        Extract<Res<crate::robomaster::combat::telemetry::CombatTelemetry>>,
    ),
    calibration: Extract<Res<TalosCameraCalibration>>,
    robots: Extract<Query<(Entity, &GlobalTransform, &Infantry, Option<&RobotIdentity>)>>,
    chassis: Extract<Query<(&GlobalTransform, &InfantryChassis)>>,
    armor_roots: Extract<Query<(Entity, &ArmorRoot, &ArmorParts)>>,
    armor_vertices: Extract<Query<(&GlobalTransform, &VertexData)>>,
    child_of: Extract<Query<&ChildOf>>,
    children: Extract<Query<&Children>>,
    names: Extract<Query<&Name>>,
    global_transforms: Extract<Query<&GlobalTransform>>,
) {
    let (frame_stamp, round, fence) = stamps;
    pose_data.round_id = round.id;
    pose_data.fence = Some(fence.clone());
    pose_data.frame_seq = frame_stamp.frame_seq;
    pose_data.timestamp_ns = frame_stamp.timestamp_ns;

    let Ok(cam_transform) = camera.single() else {
        pose_data.pose = None;
        pose_data.valid = false;
        return;
    };
    let Ok(gimbal_transform) = gimbal.single() else {
        pose_data.pose = None;
        pose_data.valid = false;
        return;
    };
    let Ok((muzzle_global, muzzle_local)) = muzzle_offset.single() else {
        pose_data.pose = None;
        pose_data.valid = false;
        return;
    };

    pose_data.pose = Some(captured_pose_data(
        cam_transform,
        gimbal_transform,
        muzzle_global,
        muzzle_local,
        **telemetry.0,
        &telemetry.1,
        &chassis_obs,
        calibration.0,
        &robots,
        &chassis,
        &armor_roots,
        &armor_vertices,
        &child_of,
        &children,
        &names,
        &global_transforms,
        pose_data.frame_seq,
        pose_data.timestamp_ns,
    ));
    pose_data.pose.as_mut().unwrap().combat = telemetry.2.frame;
    pose_data.valid = true;
}

fn captured_pose_data(
    cam_transform: &GlobalTransform,
    gimbal_transform: &GlobalTransform,
    muzzle_global: &GlobalTransform,
    muzzle_local: &Transform,
    actuator: GimbalActuatorTelemetry,
    projectile_statistics: &ProjectileStatistics,
    chassis_obs: &ChassisObservationFrame,
    camera_info: CameraInfo,
    robots: &Query<(Entity, &GlobalTransform, &Infantry, Option<&RobotIdentity>)>,
    chassis: &Query<(&GlobalTransform, &InfantryChassis)>,
    armor_roots: &Query<(Entity, &ArmorRoot, &ArmorParts)>,
    armor_vertices: &Query<(&GlobalTransform, &VertexData)>,
    child_of: &Query<&ChildOf>,
    children: &Query<&Children>,
    names: &Query<&Name>,
    global_transforms: &Query<&GlobalTransform>,
    frame_seq: u64,
    timestamp_ns: u64,
) -> CapturedPoseData {
    let shot_rotation = gimbal_transform.rotation() * muzzle_local.rotation;
    let world_t_gimbal = transform_from_axes(
        to_ros_translation(gimbal_transform.translation()),
        to_ros_translation(shot_rotation * Vec3::Y),
        to_ros_translation(shot_rotation * -Vec3::X),
        to_ros_translation(shot_rotation * Vec3::Z),
    );

    let camera_rotation = cam_transform.rotation();
    let world_t_camera = transform_from_axes(
        to_ros_translation(cam_transform.translation()),
        to_ros_translation(camera_rotation * Vec3::X),
        to_ros_translation(camera_rotation * -Vec3::Y),
        to_ros_translation(camera_rotation * -Vec3::Z),
    );
    let gimbal_t_camera_optical = relative_transform(world_t_gimbal, world_t_camera);

    let world_muzzle = to_ros_translation(muzzle_global.translation());
    let gimbal_rotation = quat_from_wire(world_t_gimbal.rotation);
    let muzzle_translation =
        gimbal_rotation.inverse() * (world_muzzle - Vec3::from_array(world_t_gimbal.translation));
    let gimbal_t_muzzle = RigidTransformF32 {
        translation: muzzle_translation.to_array(),
        rotation: QuaternionF32 {
            w: 1.0,
            ..default()
        },
        ..default()
    };

    let recomposed_camera = compose_transform(world_t_gimbal, gimbal_t_camera_optical);
    debug_assert!(transform_near(recomposed_camera, world_t_camera, 1.0e-4));

    CapturedPoseData {
        combat: default(),
        camera_info,
        world_t_gimbal,
        gimbal_t_camera_optical,
        gimbal_t_muzzle,
        actuator,
        projectile_statistics: ProjectileStatisticsMeta {
            timestamp_ns,
            bullet_launch_count: projectile_statistics.bullet_launch_count,
            armor_hit_count: projectile_statistics.armor_hit_count,
            rune_hit_count: projectile_statistics.rune_hit_count,
            dart_launch_count: projectile_statistics.dart_launch_count,
        },
        chassis_observation: ChassisObservation {
            frame_seq,
            timestamp_ns,
            dt_s: chassis_obs.dt_s,
            v_body: [chassis_obs.v_body.x, chassis_obs.v_body.y],
            wz_radps: chassis_obs.wz_radps,
            wheel_linear_mps: chassis_obs.wheel_linear_mps,
            wheel_angular_radps: chassis_obs.wheel_angular_radps,
            a_body: [chassis_obs.a_body.x, chassis_obs.a_body.y],
            alpha_z_radps2: chassis_obs.alpha_z_radps2,
            rpy_rad: [
                chassis_obs.rpy_rad.x,
                chassis_obs.rpy_rad.y,
                chassis_obs.rpy_rad.z,
            ],
            gyro_xyz_radps: [
                chassis_obs.gyro_xyz_radps.x,
                chassis_obs.gyro_xyz_radps.y,
                chassis_obs.gyro_xyz_radps.z,
            ],
            accel_xyz_mps2: [
                chassis_obs.accel_xyz_mps2.x,
                chassis_obs.accel_xyz_mps2.y,
                chassis_obs.accel_xyz_mps2.z,
            ],
            _pad: [0; 16],
        },
        ground_truth: capture_ground_truth(
            robots,
            chassis,
            armor_roots,
            armor_vertices,
            child_of,
            children,
            names,
            global_transforms,
            frame_seq,
            timestamp_ns,
        ),
    }
}
