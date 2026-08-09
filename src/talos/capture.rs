use crate::capture::{
    CameraFov, CaptureBundle, CaptureSource, ImageHandle, compute_camera_intrinsics,
    driver::{
        CaptureConfig, CaptureFrameId, CapturedFrame, CapturedFrameKind, GpuCaptureHandler,
        SnapshotAsync, SnapshotSync,
    },
    setup_capture_camera, setup_preview_window, sync_capture_camera,
};
use crate::components::{
    Controlled, Infantry, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::robomaster::prelude::{ArmorRoot, Team};
use crate::systems::{ChassisObservationFrame, GameplaySystems};
use crate::talos::plugin::{to_ros_quat, to_ros_translation};
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
}

/// Pose data captured at frame snapshot time
#[derive(Clone)]
struct CapturedPoseData {
    camera_info: CameraInfo,
    world_t_gimbal: RigidTransformF32,
    gimbal_t_camera_optical: RigidTransformF32,
    gimbal_t_muzzle: RigidTransformF32,
    chassis_observation: ChassisObservation,
    ground_truth: GroundTruthBatch,
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

        if let Ok(mut publisher) = self.ctx.lock() {
            let mut camera_info = self.pose.camera_info;
            camera_info.timestamp_ns = self.timestamp_ns;
            let metadata = CapturedFrameMeta {
                frame_seq: self.frame_seq,
                capture_timestamp_ns: self.timestamp_ns,
                camera_info,
                world_t_gimbal: self.pose.world_t_gimbal,
                gimbal_t_camera_optical: self.pose.gimbal_t_camera_optical,
                gimbal_t_muzzle: self.pose.gimbal_t_muzzle,
                chassis_observation: self.pose.chassis_observation,
                ground_truth: self.pose.ground_truth,
                ..default()
            };
            let _ = publisher.try_publish_frame(frame.data, metadata);
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
) {
    let Some(ctx) = context else {
        return;
    };

    if let Ok(mut publisher) = ctx.publisher.lock() {
        publisher.publish_runtime_state(RuntimeState {
            timestamp_ns: frame_stamp.timestamp_ns,
            following: u8::from(following.load(Ordering::Acquire)),
            _pad: [0; 55],
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
    frame_stamp: Extract<Res<TalosFrameStamp>>,
    camera: Extract<Query<&GlobalTransform, With<CaptureSource>>>,
    gimbal: Extract<Query<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>>,
    muzzle_offset: Extract<
        Query<(&GlobalTransform, &Transform), (With<InfantryLaunchOffset>, With<Controlled>)>,
    >,
    chassis_obs: Extract<Res<ChassisObservationFrame>>,
    calibration: Extract<Res<TalosCameraCalibration>>,
    robots: Extract<Query<(Entity, &GlobalTransform, &Infantry)>>,
    chassis: Extract<Query<(&GlobalTransform, &InfantryChassis)>>,
    armor_roots: Extract<Query<(Entity, &ArmorRoot)>>,
    children: Extract<Query<&Children>>,
    names: Extract<Query<&Name>>,
    global_transforms: Extract<Query<&GlobalTransform>>,
) {
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
        &chassis_obs,
        calibration.0,
        &robots,
        &chassis,
        &armor_roots,
        &children,
        &names,
        &global_transforms,
        pose_data.frame_seq,
        pose_data.timestamp_ns,
    ));
    pose_data.valid = true;
}

fn captured_pose_data(
    cam_transform: &GlobalTransform,
    gimbal_transform: &GlobalTransform,
    muzzle_global: &GlobalTransform,
    muzzle_local: &Transform,
    chassis_obs: &ChassisObservationFrame,
    camera_info: CameraInfo,
    robots: &Query<(Entity, &GlobalTransform, &Infantry)>,
    chassis: &Query<(&GlobalTransform, &InfantryChassis)>,
    armor_roots: &Query<(Entity, &ArmorRoot)>,
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
        camera_info,
        world_t_gimbal,
        gimbal_t_camera_optical,
        gimbal_t_muzzle,
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
            children,
            names,
            global_transforms,
            frame_seq,
            timestamp_ns,
        ),
    }
}

fn wire_quaternion(rotation: Quat) -> QuaternionF32 {
    let q = rotation.normalize();
    QuaternionF32 {
        x: q.x,
        y: q.y,
        z: q.z,
        w: q.w,
    }
}

fn quat_from_wire(rotation: QuaternionF32) -> Quat {
    Quat::from_xyzw(rotation.x, rotation.y, rotation.z, rotation.w)
}

fn transform_from_axes(origin: Vec3, x: Vec3, y: Vec3, z: Vec3) -> RigidTransformF32 {
    let rotation = Quat::from_mat3(&Mat3::from_cols(
        x.normalize(),
        y.normalize(),
        z.normalize(),
    ));
    RigidTransformF32 {
        translation: origin.to_array(),
        rotation: wire_quaternion(rotation),
        ..default()
    }
}

fn relative_transform(parent: RigidTransformF32, child: RigidTransformF32) -> RigidTransformF32 {
    let parent_rotation = quat_from_wire(parent.rotation);
    let child_rotation = quat_from_wire(child.rotation);
    let translation = parent_rotation.inverse()
        * (Vec3::from_array(child.translation) - Vec3::from_array(parent.translation));
    RigidTransformF32 {
        translation: translation.to_array(),
        rotation: wire_quaternion(parent_rotation.inverse() * child_rotation),
        ..default()
    }
}

fn compose_transform(parent: RigidTransformF32, child: RigidTransformF32) -> RigidTransformF32 {
    let parent_rotation = quat_from_wire(parent.rotation);
    RigidTransformF32 {
        translation: (Vec3::from_array(parent.translation)
            + parent_rotation * Vec3::from_array(child.translation))
        .to_array(),
        rotation: wire_quaternion(parent_rotation * quat_from_wire(child.rotation)),
        ..default()
    }
}

fn transform_near(left: RigidTransformF32, right: RigidTransformF32, tolerance: f32) -> bool {
    let translation_error =
        Vec3::from_array(left.translation).distance(Vec3::from_array(right.translation));
    let rotation_error =
        quat_from_wire(left.rotation).angle_between(quat_from_wire(right.rotation));
    translation_error <= tolerance && rotation_error <= tolerance
}

fn capture_ground_truth(
    robots: &Query<(Entity, &GlobalTransform, &Infantry)>,
    chassis: &Query<(&GlobalTransform, &InfantryChassis)>,
    armor_roots: &Query<(Entity, &ArmorRoot)>,
    children: &Query<&Children>,
    names: &Query<&Name>,
    global_transforms: &Query<&GlobalTransform>,
    frame_seq: u64,
    timestamp_ns: u64,
) -> GroundTruthBatch {
    let mut batch = GroundTruthBatch {
        frame_seq,
        timestamp_ns,
        ..default()
    };

    for (entity, transform, infantry) in robots.iter() {
        if batch.target_count as usize >= GROUND_TRUTH_MAX_TARGETS {
            break;
        }
        let position = to_ros_translation(transform.translation());
        // Infantry 位于机器人根实体，但车身朝向实际施加在其 BASE/InfantryChassis
        // 子实体上。使用子实体的全局姿态，避免所有根实体的单位旋转被导出为 yaw=0。
        let (heading_transform, vyaw) = find_chassis_descendant(entity, children, chassis)
            .and_then(|chassis_entity| chassis.get(chassis_entity).ok())
            .map(|(chassis_transform, chassis_state)| {
                (chassis_transform, chassis_state.yaw_velocity)
            })
            .unwrap_or((transform, 0.0));
        let rotation = to_ros_quat(heading_transform.rotation());
        let (_, _, yaw) = rotation.to_euler(EulerRot::XYZ);
        let index = batch.target_count as usize;
        batch.targets[index] = GroundTruthTarget {
            frame_seq,
            timestamp_ns,
            id: entity.to_bits(),
            team: match infantry.team {
                Team::Red => 0,
                Team::Blue => 1,
            },
            armor_label: infantry.config.armor.label() as u8,
            position: position.to_array(),
            vyaw,
            yaw,
            ..default()
        };
        batch.target_count += 1;
    }

    for (entity, armor) in armor_roots.iter() {
        if batch.projection_probe_count as usize >= PROJECTION_PROBE_MAX_COUNT {
            break;
        }
        let Some(center) = find_named_suffix_descendant(entity, "CENTER", children, names) else {
            continue;
        };
        let Ok(center_transform) = global_transforms.get(center) else {
            continue;
        };
        let index = batch.projection_probe_count as usize;
        batch.projection_probes[index] = ProjectionProbe {
            id: (entity.to_bits() << 8) ^ armor.id.as_usize() as u64,
            position_world: to_ros_translation(center_transform.translation()).to_array(),
            ..default()
        };
        batch.projection_probe_count += 1;
    }
    batch
}

fn find_chassis_descendant(
    root: Entity,
    children: &Query<&Children>,
    chassis: &Query<(&GlobalTransform, &InfantryChassis)>,
) -> Option<Entity> {
    let mut pending = vec![root];
    while let Some(entity) = pending.pop() {
        if chassis.get(entity).is_ok() {
            return Some(entity);
        }
        if let Ok(descendants) = children.get(entity) {
            pending.extend(descendants.iter());
        }
    }
    None
}

fn find_named_suffix_descendant(
    root: Entity,
    wanted: &str,
    children: &Query<&Children>,
    names: &Query<&Name>,
) -> Option<Entity> {
    let mut pending = vec![root];
    while let Some(entity) = pending.pop() {
        // GLB 节点带有装甲序号和规格前缀，例如 `1__L_ARMOR_CENTER`。
        if names
            .get(entity)
            .is_ok_and(|name| name.as_str().ends_with(wanted))
        {
            return Some(entity);
        }
        if let Ok(descendants) = children.get(entity) {
            pending.extend(descendants.iter());
        }
    }
    None
}
