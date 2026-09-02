//! A frame pool over one caller-owned buffer.
//!
//! A camera DMA engine fills one slot while the encoder reads another. On a
//! chip the buffer is a static array (often in PSRAM); on the host it is a
//! `Vec`. The pool never allocates and never hands out overlapping memory:
//! slots are addressed by [`SlotId`], and the pool is borrowed for exactly as
//! long as a slot's bytes are being touched.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::{Frame, Geometry};
use rusty_esp_core::time::Micros;

/// Index of a slot in a [`FramePool`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SlotId(u8);

impl SlotId {
    /// The index.
    #[must_use]
    pub fn index(self) -> usize {
        usize::from(self.0)
    }
}

/// `N` equal slots carved from one buffer.
#[derive(Debug)]
pub struct FramePool<'m, const N: usize> {
    buf: &'m mut [u8],
    slot_len: usize,
    in_use: [bool; N],
    used: [usize; N],
}

impl<'m, const N: usize> FramePool<'m, N> {
    /// Split `buf` into `N` slots. `N` must be 1..=255 and each slot at least
    /// one byte; the trailing remainder of `buf` is unused.
    pub fn new(buf: &'m mut [u8]) -> Result<Self> {
        if N == 0 || N > 255 {
            return Err(Error::Unsupported);
        }
        let slot_len = buf.len() / N;
        if slot_len == 0 {
            return Err(Error::BufferTooSmall { needed: N });
        }
        Ok(FramePool {
            buf,
            slot_len,
            in_use: [false; N],
            used: [0; N],
        })
    }

    /// Bytes per slot.
    #[must_use]
    pub fn slot_len(&self) -> usize {
        self.slot_len
    }

    /// Slots not currently acquired.
    #[must_use]
    pub fn free_slots(&self) -> usize {
        self.in_use.iter().filter(|b| !**b).count()
    }

    /// Take a free slot, or `None` when all are in use (a dropped frame, by
    /// policy of the caller — never a block).
    pub fn acquire(&mut self) -> Option<SlotId> {
        let i = self.in_use.iter().position(|b| !*b)?;
        self.in_use[i] = true;
        self.used[i] = 0;
        Some(SlotId(i as u8))
    }

    /// Return a slot to the pool.
    pub fn release(&mut self, id: SlotId) -> Result<()> {
        let i = self.check(id)?;
        self.in_use[i] = false;
        self.used[i] = 0;
        Ok(())
    }

    /// The whole slot, for filling.
    pub fn slot_mut(&mut self, id: SlotId) -> Result<&mut [u8]> {
        let i = self.check(id)?;
        let start = i * self.slot_len;
        Ok(&mut self.buf[start..start + self.slot_len])
    }

    /// Record how many bytes of the slot hold the frame (a JPEG is shorter
    /// than its slot; a raw frame fills exactly its geometry).
    pub fn commit(&mut self, id: SlotId, used: usize) -> Result<()> {
        let i = self.check(id)?;
        if used > self.slot_len {
            return Err(Error::BufferTooSmall { needed: used });
        }
        self.used[i] = used;
        Ok(())
    }

    /// The committed bytes of a slot.
    pub fn slot(&self, id: SlotId) -> Result<&[u8]> {
        let i = self.check(id)?;
        let start = i * self.slot_len;
        Ok(&self.buf[start..start + self.used[i]])
    }

    /// A validated frame view over the committed bytes of a slot.
    pub fn frame(
        &self,
        id: SlotId,
        geometry: Geometry,
        timestamp: Micros,
        sequence: u32,
    ) -> Result<Frame<'_>> {
        Frame::packed(geometry, timestamp, sequence, self.slot(id)?)
    }

    fn check(&self, id: SlotId) -> Result<usize> {
        let i = id.index();
        if i >= N || !self.in_use[i] {
            return Err(Error::InvalidFormat);
        }
        Ok(i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusty_esp_core::frame::PixelFormat;

    #[test]
    fn acquire_fill_commit_frame_release() {
        let mut mem = [0u8; 64];
        let mut pool: FramePool<'_, 2> = FramePool::new(&mut mem).unwrap();
        assert_eq!(pool.slot_len(), 32);
        assert_eq!(pool.free_slots(), 2);
        let a = pool.acquire().unwrap();
        let b = pool.acquire().unwrap();
        assert!(pool.acquire().is_none(), "no blocking, no third slot");

        // "DMA" writes a JPEG into slot a
        let s = pool.slot_mut(a).unwrap();
        s[..4].copy_from_slice(&[0xFF, 0xD8, 0xFF, 0xD9]);
        pool.commit(a, 4).unwrap();
        let g = Geometry::new(320, 240, PixelFormat::Jpeg).unwrap();
        let f = pool.frame(a, g, Micros::from_millis(3), 7).unwrap();
        assert_eq!(f.coded().map(<[u8]>::len), Some(4));
        assert_eq!(f.sequence, 7);

        // a raw frame that does not fit its geometry is refused
        let g8 = Geometry::new(8, 8, PixelFormat::Gray8).unwrap();
        pool.commit(b, 10).unwrap();
        assert!(matches!(
            pool.frame(b, g8, Micros::ZERO, 0),
            Err(Error::BufferTooSmall { needed: 64 })
        ));

        pool.release(a).unwrap();
        assert_eq!(pool.free_slots(), 1);
        assert_eq!(pool.release(a), Err(Error::InvalidFormat), "double release");
        assert_eq!(
            pool.commit(b, 33),
            Err(Error::BufferTooSmall { needed: 33 })
        );
    }

    #[test]
    fn construction_rules() {
        let mut tiny = [0u8; 1];
        assert!(matches!(
            FramePool::<'_, 2>::new(&mut tiny),
            Err(Error::BufferTooSmall { needed: 2 })
        ));
        let mut ok = [0u8; 3];
        let pool: FramePool<'_, 2> = FramePool::new(&mut ok).unwrap();
        assert_eq!(pool.slot_len(), 1);
    }
}
