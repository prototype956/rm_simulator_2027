use crate::layout::*;
use crate::shm::{ShmError, ShmRegion};
use crate::triple_buffer::TripleBufferProducer;
use std::sync::atomic::Ordering;
use std::time::{SystemTime, UNIX_EPOCH};

pub struct ShmPublisher {
    meta_region: ShmRegion,
    image_pool: ShmRegion,
    current_buffer_id: u8,
    last_synchronized_frame_seq: Option<u64>,
    image_width: u32,
    image_height: u32,
    image_format: ImageFormat,
    image_size: usize,
}

impl ShmPublisher {
    pub fn create() -> Result<Self, ShmError> {
        Self::create_with_image(IMAGE_WIDTH, IMAGE_HEIGHT, ImageFormat::Rgb8)
    }

    pub fn create_with_image(
        image_width: u32,
        image_height: u32,
        image_format: ImageFormat,
    ) -> Result<Self, ShmError> {
        if image_width == 0 || image_height == 0 {
            return Err(ShmError::InvalidSize);
        }
        let image_size =
            image_size(image_width, image_height, image_format).ok_or(ShmError::InvalidSize)?;
        let pool_size = image_pool_size(image_width, image_height, image_format)
            .ok_or(ShmError::InvalidSize)?;
        let mut meta_region = ShmRegion::create(SHM_NAME_META, size_of::<ShmMetaRegion>())?;
        let image_pool = ShmRegion::create(SHM_NAME_IMAGE_POOL, pool_size)?;

        unsafe {
            let meta = meta_region.as_mut::<ShmMetaRegion>();

            // 初始化 header
            meta.header = ShmHeader {
                magic: SHM_MAGIC,
                version: SHM_VERSION,
                created_ns: Self::now_ns(),
                heartbeat_ns: Self::now_ns(),
                image_width,
                image_height,
                _pad: [0; 32],
            };

            // 初始化所有 TripleBuffer (CRITICAL: 零填充破坏了正确的初始状态)
            // 正确初始状态: state=1 (ready slot), write_idx=0, read_idx=2
            Self::init_triple_buffer(&mut meta.image);
            for pose in &mut meta.poses {
                Self::init_triple_buffer(pose);
            }
            Self::init_triple_buffer(&mut meta.gimbal_cmd);
        }

        Ok(Self {
            meta_region,
            image_pool,
            current_buffer_id: 0,
            last_synchronized_frame_seq: None,
            image_width,
            image_height,
            image_format,
            image_size,
        })
    }

    pub fn publish_image(&mut self, data: &[u8], seq: u64, timestamp_ns: u64) {
        self.publish_image_with(data, seq, timestamp_ns, |_| {});
    }

    pub fn publish_image_with<F>(
        &mut self,
        data: &[u8],
        seq: u64,
        timestamp_ns: u64,
        before_commit: F,
    ) where
        F: FnOnce(&mut Self),
    {
        assert_eq!(data.len(), self.image_size, "Image size mismatch");

        let buffer_id = self.current_buffer_id;
        self.current_buffer_id = (self.current_buffer_id + 1) % 3;

        unsafe {
            let pool_ptr = self.image_pool.as_ptr();
            let dst = pool_ptr.add(buffer_id as usize * self.image_size);
            std::ptr::copy_nonoverlapping(data.as_ptr(), dst, self.image_size);
        }

        // Publish data associated with this image only after the expensive pixel copy. The image
        // metadata below is the commit marker observed by consumers.
        before_commit(self);

        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let mut producer = TripleBufferProducer::new(
                &meta.image.state,
                &mut meta.image.write_idx,
                &mut meta.image.slots,
            );

            let slot = producer.borrow_mut();
            slot.seq = seq;
            slot.timestamp_ns = timestamp_ns;
            slot.width = self.image_width;
            slot.height = self.image_height;
            slot.buffer_id = buffer_id;
            slot.format = self.image_format as u8;
            producer.publish();
        }
    }

    #[must_use]
    pub fn try_publish_synchronized_image<F>(
        &mut self,
        data: &[u8],
        seq: u64,
        timestamp_ns: u64,
        before_commit: F,
    ) -> bool
    where
        F: FnOnce(&mut Self),
    {
        // Never publish an older async readback, and never overwrite one half of an image/pose
        // bundle while the consumer is between the two triple buffers.
        if self
            .last_synchronized_frame_seq
            .is_some_and(|last_seq| seq <= last_seq)
            || !self.synchronized_frame_consumed()
        {
            return false;
        }

        self.publish_image_with(data, seq, timestamp_ns, before_commit);
        self.last_synchronized_frame_seq = Some(seq);
        true
    }

    fn synchronized_frame_consumed(&self) -> bool {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            let image_consumed = meta.image.state.load(Ordering::Acquire) & FLAG_NEW == 0;
            // Gimbal, odom, muzzle and camera are consumed with each image. Slot 4 is the legacy
            // chassis-observation channel and is intentionally not part of this handshake.
            let poses_consumed = meta.poses[..=PoseIndex::Camera as usize]
                .iter()
                .all(|pose| pose.state.load(Ordering::Acquire) & FLAG_NEW == 0);
            image_consumed && poses_consumed
        }
    }

    pub fn publish_pose(
        &mut self,
        index: PoseIndex,
        position: [f32; 3],
        quaternion: [f32; 4],
        frame_seq: u64,
        timestamp_ns: u64,
    ) {
        self.publish_pose_with_aux(
            index,
            position,
            quaternion,
            [0.0; 4],
            frame_seq,
            timestamp_ns,
        );
    }

    pub fn publish_pose_with_aux(
        &mut self,
        index: PoseIndex,
        position: [f32; 3],
        quaternion: [f32; 4],
        aux_f32: [f32; 4],
        frame_seq: u64,
        timestamp_ns: u64,
    ) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let pose_buf = &mut meta.poses[index as usize];
            let mut producer = TripleBufferProducer::new(
                &pose_buf.state,
                &mut pose_buf.write_idx,
                &mut pose_buf.slots,
            );

            let slot = producer.borrow_mut();
            slot.frame_seq = frame_seq;
            slot.position = position;
            slot.quaternion = quaternion;
            slot.timestamp_ns = timestamp_ns;
            slot._pad = aux_f32_to_bytes(aux_f32);

            producer.publish();
        }
    }

    pub fn set_camera_info(&mut self, info: CameraInfo) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.camera_info = info;
        }
    }

    pub fn publish_chassis_observation(&mut self, observation: ChassisObservation) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.chassis_observation = observation;
        }
    }

    pub fn publish_ground_truth(&mut self, batch: &GroundTruthBatch) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.ground_truth = *batch;
        }
    }

    pub fn publish_runtime_state(&mut self, state: RuntimeState) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.runtime_state = state;
        }
    }

    pub fn update_heartbeat(&mut self) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.header.heartbeat_ns = Self::now_ns();
        }
    }

    fn now_ns() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0)
    }

    /// 初始化 TripleBuffer 到正确的初始状态
    ///
    /// ShmRegion::create() 使用零填充，会破坏 TripleBuffer 的正确初始状态。
    /// 必须手动重新初始化。
    ///
    /// 正确初始状态:
    /// - state = 1 (ready slot 是 1, 无 FLAG_NEW)
    /// - write_idx = 0 (生产者写入 slot 0)
    /// - read_idx = 2 (消费者上次读取 slot 2)
    fn init_triple_buffer(buf: &mut impl TripleBufferInit) {
        buf.init_state();
    }
}

fn aux_f32_to_bytes(aux_f32: [f32; 4]) -> [u8; 16] {
    let mut bytes = [0u8; 16];
    for (i, value) in aux_f32.iter().enumerate() {
        bytes[i * 4..(i + 1) * 4].copy_from_slice(&value.to_le_bytes());
    }
    bytes
}

/// Trait for initializing triple buffer state
trait TripleBufferInit {
    fn init_state(&mut self);
}

impl TripleBufferInit for ImageTripleBuffer {
    fn init_state(&mut self) {
        self.state.store(1, Ordering::Relaxed);
        self.write_idx = 0;
        self.read_idx = 2;
    }
}

impl TripleBufferInit for PoseTripleBuffer {
    fn init_state(&mut self) {
        self.state.store(1, Ordering::Relaxed);
        self.write_idx = 0;
        self.read_idx = 2;
    }
}

impl TripleBufferInit for GimbalTripleBuffer {
    fn init_state(&mut self) {
        self.state.store(1, Ordering::Relaxed);
        self.write_idx = 0;
        self.read_idx = 2;
    }
}
