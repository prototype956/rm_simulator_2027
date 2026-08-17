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
            Self::init_triple_buffer(&mut meta.frame);
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

    pub fn publish_frame(&mut self, data: &[u8], mut frame: CapturedFrameMeta) {
        assert_eq!(data.len(), self.image_size, "Image size mismatch");

        let buffer_id = self.current_buffer_id;
        self.current_buffer_id = (self.current_buffer_id + 1) % 3;

        unsafe {
            let pool_ptr = self.image_pool.as_ptr();
            let dst = pool_ptr.add(buffer_id as usize * self.image_size);
            std::ptr::copy_nonoverlapping(data.as_ptr(), dst, self.image_size);
        }

        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let mut producer = TripleBufferProducer::new(
                &meta.frame.state,
                &mut meta.frame.write_idx,
                &mut meta.frame.slots,
            );

            let slot = producer.borrow_mut();
            frame.width = self.image_width;
            frame.height = self.image_height;
            frame.buffer_id = buffer_id;
            frame.format = self.image_format as u8;
            *slot = frame;
            producer.publish();
        }
    }

    #[must_use]
    pub fn try_publish_frame(&mut self, data: &[u8], frame: CapturedFrameMeta) -> bool {
        if self
            .last_synchronized_frame_seq
            .is_some_and(|last_seq| frame.frame_seq <= last_seq)
            || !self.frame_consumed()
        {
            return false;
        }

        let frame_seq = frame.frame_seq;
        self.publish_frame(data, frame);
        self.last_synchronized_frame_seq = Some(frame_seq);
        true
    }

    fn frame_consumed(&self) -> bool {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            meta.frame.state.load(Ordering::Acquire) & FLAG_NEW == 0
        }
    }

    pub fn publish_runtime_state(&mut self, state: RuntimeState) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let timestamp = state.timestamp_ns;
            let timestamp_ptr = std::ptr::addr_of_mut!(meta.runtime_state.timestamp_ns);
            std::sync::atomic::AtomicU64::from_ptr(timestamp_ptr).store(0, Ordering::Release);
            let mut pending = state;
            pending.timestamp_ns = 0;
            std::ptr::write_volatile(std::ptr::addr_of_mut!(meta.runtime_state), pending);
            std::sync::atomic::fence(Ordering::Release);
            std::sync::atomic::AtomicU64::from_ptr(timestamp_ptr)
                .store(timestamp, Ordering::Release);
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

/// Trait for initializing triple buffer state
trait TripleBufferInit {
    fn init_state(&mut self);
}

impl TripleBufferInit for FrameTripleBuffer {
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
