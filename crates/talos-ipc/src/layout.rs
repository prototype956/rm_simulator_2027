use std::sync::atomic::AtomicU8;

pub const IMAGE_WIDTH: u32 = 1440;
pub const IMAGE_HEIGHT: u32 = 1080;

pub const CACHE_LINE_SIZE: usize = 64;
pub const SHM_MAGIC: u32 = 0x54414C05;
pub const SHM_VERSION: u32 = 5;

pub const IMAGE_CHANNELS: u32 = 3;
pub const IMAGE_SIZE: usize = (IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS) as usize;
pub const IMAGE_POOL_SIZE: usize = IMAGE_SIZE * 3;
pub const SHM_NAME_META: &str = "talos_ipc_meta";
pub const SHM_NAME_IMAGE_POOL: &str = "talos_ipc_image_pool";

pub const FLAG_NEW: u8 = 0x80;
pub const INDEX_MASK: u8 = 0x03;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    Rgb8 = 0,
    Bgr8 = 1,
}

impl ImageFormat {
    pub const fn channels(self) -> usize {
        match self {
            Self::Rgb8 | Self::Bgr8 => IMAGE_CHANNELS as usize,
        }
    }
}

pub fn image_size(width: u32, height: u32, format: ImageFormat) -> Option<usize> {
    (width as usize)
        .checked_mul(height as usize)?
        .checked_mul(format.channels())
}

pub fn image_pool_size(width: u32, height: u32, format: ImageFormat) -> Option<usize> {
    image_size(width, height, format)?.checked_mul(3)
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct QuaternionF32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
const _: () = assert!(size_of::<QuaternionF32>() == 16);

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy, Default)]
pub struct RigidTransformF32 {
    pub translation: [f32; 3],
    pub rotation: QuaternionF32,
    pub _pad: [u8; 4],
}
const _: () = assert!(size_of::<RigidTransformF32>() == 32);

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy, Default)]
pub struct GimbalCmd {
    pub timestamp_ns: u64,
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub distance_m: f32,
    pub fire_advice: u8,
    pub _pad: [u8; 11],
}
const _: () = assert!(size_of::<GimbalCmd>() == 32);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy, Default)]
pub struct CameraInfo {
    pub timestamp_ns: u64,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion: [f64; 5],
    pub width: u32,
    pub height: u32,
    pub _pad: [u8; 24],
}
const _: () = assert!(size_of::<CameraInfo>() == 128);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct ChassisObservation {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub dt_s: f32,
    pub v_body: [f32; 2],
    pub wz_radps: f32,
    pub wheel_linear_mps: [f32; 4],
    pub wheel_angular_radps: [f32; 4],
    pub a_body: [f32; 2],
    pub alpha_z_radps2: f32,
    pub rpy_rad: [f32; 3],
    pub gyro_xyz_radps: [f32; 3],
    pub accel_xyz_mps2: [f32; 3],
    pub _pad: [u8; 16],
}
const _: () = assert!(size_of::<ChassisObservation>() == 128);

impl Default for ChassisObservation {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            timestamp_ns: 0,
            dt_s: 0.0,
            v_body: [0.0; 2],
            wz_radps: 0.0,
            wheel_linear_mps: [0.0; 4],
            wheel_angular_radps: [0.0; 4],
            a_body: [0.0; 2],
            alpha_z_radps2: 0.0,
            rpy_rad: [0.0; 3],
            gyro_xyz_radps: [0.0; 3],
            accel_xyz_mps2: [0.0; 3],
            _pad: [0; 16],
        }
    }
}

#[repr(C, align(64))]
pub struct GimbalTripleBuffer {
    pub state: AtomicU8,
    pub write_idx: u8,
    pub read_idx: u8,
    pub _pad1: [u8; 61],
    pub slots: [GimbalCmd; 3],
}
const _: () = assert!(size_of::<GimbalTripleBuffer>() == 192);

#[repr(C, align(64))]
pub struct ShmHeader {
    pub magic: u32,
    pub version: u32,
    pub created_ns: u64,
    pub heartbeat_ns: u64,
    pub image_width: u32,
    pub image_height: u32,
    pub _pad: [u8; 32],
}
const _: () = assert!(size_of::<ShmHeader>() == 64);

pub const GROUND_TRUTH_MAX_TARGETS: usize = 16;
pub const GROUND_TRUTH_MAX_RUNES: usize = 4;
pub const GROUND_TRUTH_MAX_ARMORS: usize = 32;

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy, Default)]
pub struct GroundTruthTarget {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub id: u64,
    pub team: u8,
    pub armor_label: u8,
    pub is_outpost: u8,
    pub _pad1: u8,
    pub position: [f32; 3],
    pub vyaw: f32,
    pub yaw: f32,
    pub _pad: [u8; 16],
}
const _: () = assert!(size_of::<GroundTruthTarget>() == 64);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy, Default)]
pub struct GroundTruthArmor {
    pub id: u64,
    pub team: u8,
    pub label: u8,
    pub armor_type: u8,
    pub _pad1: u8,
    pub width_m: f32,
    pub height_m: f32,
    pub _pad2: [u8; 12],
    pub world_t_armor: RigidTransformF32,
    /// TL/TR/BR/BL light-bar endpoints in the ROS world frame.
    pub corners_world: [[f32; 3]; 4],
    pub _pad3: [u8; 16],
}
const _: () = assert!(size_of::<GroundTruthArmor>() == 128);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct GroundTruthRune {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub team: u8,
    pub rune_mode: u8,
    pub mechanism_state: u8,
    pub _pad1: u8,
    pub r_center_odom: [f32; 3],
    pub radius: f32,
    pub current_angle: f32,
    pub v_roll: f32,
    pub direction: i32,
    pub sin_amplitude: f32,
    pub sin_omega: f32,
    pub sin_phase: f32,
    pub sin_offset: f32,
    pub relative_time: f32,
    pub blade_id: i32,
    pub target_activations: [u8; 5],
    pub _pad: [u8; 20],
}
const _: () = assert!(size_of::<GroundTruthRune>() == 128);

impl Default for GroundTruthRune {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            timestamp_ns: 0,
            team: 0,
            rune_mode: 0,
            mechanism_state: 0,
            _pad1: 0,
            r_center_odom: [0.0; 3],
            radius: 0.0,
            current_angle: 0.0,
            v_roll: 0.0,
            direction: 0,
            sin_amplitude: 0.0,
            sin_omega: 0.0,
            sin_phase: 0.0,
            sin_offset: 0.0,
            relative_time: 0.0,
            blade_id: -1,
            target_activations: [0; 5],
            _pad: [0; 20],
        }
    }
}

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct GroundTruthBatch {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub target_count: u32,
    pub rune_count: u32,
    pub armor_count: u32,
    pub _pad1: u32,
    pub targets: [GroundTruthTarget; GROUND_TRUTH_MAX_TARGETS],
    pub runes: [GroundTruthRune; GROUND_TRUTH_MAX_RUNES],
    pub armors: [GroundTruthArmor; GROUND_TRUTH_MAX_ARMORS],
}
const _: () = assert!(size_of::<GroundTruthBatch>() == 5696);

impl Default for GroundTruthBatch {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            timestamp_ns: 0,
            target_count: 0,
            rune_count: 0,
            armor_count: 0,
            _pad1: 0,
            targets: [GroundTruthTarget::default(); GROUND_TRUTH_MAX_TARGETS],
            runes: [GroundTruthRune::default(); GROUND_TRUTH_MAX_RUNES],
            armors: [GroundTruthArmor::default(); GROUND_TRUTH_MAX_ARMORS],
        }
    }
}

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy, Default)]
pub struct CapturedFrameMeta {
    pub frame_seq: u64,
    pub capture_timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub buffer_id: u8,
    pub format: u8,
    pub _pad1: [u8; 30],
    pub camera_info: CameraInfo,
    pub world_t_gimbal: RigidTransformF32,
    pub gimbal_t_camera_optical: RigidTransformF32,
    pub gimbal_t_muzzle: RigidTransformF32,
    pub _pad2: [u8; 32],
    pub chassis_observation: ChassisObservation,
    pub ground_truth: GroundTruthBatch,
}
const _: () = assert!(size_of::<CapturedFrameMeta>() == 6144);

#[repr(C, align(64))]
pub struct FrameTripleBuffer {
    pub state: AtomicU8,
    pub write_idx: u8,
    pub read_idx: u8,
    pub _pad1: [u8; 61],
    pub slots: [CapturedFrameMeta; 3],
}
const _: () = assert!(size_of::<FrameTripleBuffer>() == 18496);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct RuntimeState {
    pub timestamp_ns: u64,
    pub following: u8,
    pub _pad: [u8; 55],
}
const _: () = assert!(size_of::<RuntimeState>() == 64);

impl Default for RuntimeState {
    fn default() -> Self {
        Self {
            timestamp_ns: 0,
            following: 0,
            _pad: [0; 55],
        }
    }
}

#[repr(C)]
pub struct ShmMetaRegion {
    pub header: ShmHeader,
    pub frame: FrameTripleBuffer,
    pub gimbal_cmd: GimbalTripleBuffer,
    pub runtime_state: RuntimeState,
}
const _: () = assert!(size_of::<ShmMetaRegion>() == 18816);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, frame) == 64);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, gimbal_cmd) == 18560);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, runtime_state) == 18752);

impl Default for FrameTripleBuffer {
    fn default() -> Self {
        Self {
            state: AtomicU8::new(1),
            write_idx: 0,
            read_idx: 2,
            _pad1: [0; 61],
            slots: [CapturedFrameMeta::default(); 3],
        }
    }
}

impl Default for GimbalTripleBuffer {
    fn default() -> Self {
        Self {
            state: AtomicU8::new(1),
            write_idx: 0,
            read_idx: 2,
            _pad1: [0; 61],
            slots: [GimbalCmd::default(); 3],
        }
    }
}

impl Default for ShmHeader {
    fn default() -> Self {
        Self {
            magic: SHM_MAGIC,
            version: SHM_VERSION,
            created_ns: 0,
            heartbeat_ns: 0,
            image_width: IMAGE_WIDTH,
            image_height: IMAGE_HEIGHT,
            _pad: [0; 32],
        }
    }
}

impl Default for ShmMetaRegion {
    fn default() -> Self {
        Self {
            header: ShmHeader::default(),
            frame: FrameTripleBuffer::default(),
            gimbal_cmd: GimbalTripleBuffer::default(),
            runtime_state: RuntimeState::default(),
        }
    }
}
