use crate::layout::*;
use crate::shm::{ShmError, ShmRegion};
use crate::triple_buffer::TripleBufferConsumer;

pub struct ShmSubscriber {
    meta_region: ShmRegion,
}

impl ShmSubscriber {
    pub fn connect() -> Result<Self, ShmError> {
        // Read the common header before checking the larger v7 region size.
        let meta_region = ShmRegion::open(SHM_NAME_META, size_of::<ShmHeader>())?;
        unsafe {
            let header = meta_region.as_ref::<ShmHeader>();
            if header.magic != SHM_MAGIC || header.version != SHM_VERSION {
                return Err(ShmError::ProtocolMismatch {
                    actual: header.version,
                });
            }
        }
        if meta_region.size() != size_of::<ShmMetaRegion>() {
            return Err(ShmError::InvalidSize);
        }

        Ok(Self { meta_region })
    }

    pub fn recv_gimbal_cmd(&mut self) -> Option<GimbalCmd> {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let mut consumer = TripleBufferConsumer::new(
                &meta.gimbal_cmd.state,
                &mut meta.gimbal_cmd.read_idx,
                &meta.gimbal_cmd.slots,
            );

            consumer.borrow().copied()
        }
    }

    pub fn has_gimbal_cmd(&self) -> bool {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            (meta
                .gimbal_cmd
                .state
                .load(std::sync::atomic::Ordering::Acquire)
                & FLAG_NEW)
                != 0
        }
    }
}
