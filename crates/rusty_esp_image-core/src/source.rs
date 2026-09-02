//! The capture seam.
//!
//! An [`ImageSource`] writes one frame **into memory the caller owns** and
//! returns a validated view of it. A DVP engine, a MIPI-CSI engine, a file on
//! the host and the [`TestPattern`] below all fit the same two methods, so
//! everything downstream (encoders, packetizers, the mesh) is written once.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::{Frame, Geometry, PixelFormat};
use rusty_esp_core::time::Micros;

/// Something that produces frames.
pub trait ImageSource {
    /// The geometry every frame from this source has right now.
    fn geometry(&self) -> Geometry;

    /// Capture one frame into `out`. The returned view borrows `out`; for a
    /// JPEG source only the coded bytes are meaningful and `out` may be
    /// larger than the frame.
    fn grab<'b>(&mut self, out: &'b mut [u8]) -> Result<Frame<'b>>;
}

/// A deterministic synthetic source: SMPTE-style colour bars with a moving
/// marker, in Gray8, RGB565 or RGB888. It exists so a pipeline can be
/// exercised on the host, and so a firmware can prove its wiring before a
/// sensor is attached. Frame `n` is identical on every platform.
#[derive(Debug, Clone)]
pub struct TestPattern {
    geometry: Geometry,
    frame_micros: u64,
    sequence: u32,
    timestamp: Micros,
}

impl TestPattern {
    /// A pattern source at `fps` frames per second (the timestamps advance
    /// by `1_000_000 / fps` microseconds per frame). Only uncompressed packed
    /// formats are supported.
    pub fn new(geometry: Geometry, fps: u32) -> Result<Self> {
        if fps == 0 {
            return Err(Error::InvalidGeometry);
        }
        match geometry.format {
            PixelFormat::Gray8 | PixelFormat::Rgb565 | PixelFormat::Rgb888 => Ok(TestPattern {
                geometry,
                frame_micros: 1_000_000 / u64::from(fps),
                sequence: 0,
                timestamp: Micros::ZERO,
            }),
            _ => Err(Error::Unsupported),
        }
    }

    /// The seven bars, as RGB888.
    const BARS: [[u8; 3]; 7] = [
        [0xC0, 0xC0, 0xC0], // white
        [0xC0, 0xC0, 0x00], // yellow
        [0x00, 0xC0, 0xC0], // cyan
        [0x00, 0xC0, 0x00], // green
        [0xC0, 0x00, 0xC0], // magenta
        [0xC0, 0x00, 0x00], // red
        [0x00, 0x00, 0xC0], // blue
    ];

    fn colour_at(&self, x: u32, y: u32) -> [u8; 3] {
        let w = self.geometry.width;
        let h = self.geometry.height;
        // A marker row sweeps down one pixel per frame so consecutive frames differ.
        let marker_y = self.sequence % h.max(1);
        if y == marker_y {
            return [0xFF, 0xFF, 0xFF];
        }
        let bar = ((u64::from(x) * 7) / u64::from(w.max(1))) as usize;
        Self::BARS[bar.min(6)]
    }
}

impl ImageSource for TestPattern {
    fn geometry(&self) -> Geometry {
        self.geometry
    }

    fn grab<'b>(&mut self, out: &'b mut [u8]) -> Result<Frame<'b>> {
        let needed = self.geometry.byte_len().ok_or(Error::Unsupported)?;
        if out.len() < needed {
            return Err(Error::BufferTooSmall { needed });
        }
        let (w, h) = (self.geometry.width, self.geometry.height);
        let mut i = 0usize;
        for y in 0..h {
            for x in 0..w {
                let [r, g, b] = self.colour_at(x, y);
                match self.geometry.format {
                    PixelFormat::Gray8 => {
                        // BT.601 luma, integer.
                        out[i] = ((77 * u32::from(r) + 150 * u32::from(g) + 29 * u32::from(b)) >> 8)
                            as u8;
                        i += 1;
                    }
                    PixelFormat::Rgb565 => {
                        let p = crate::ops::pack_rgb565(r, g, b);
                        out[i..i + 2].copy_from_slice(&p.to_le_bytes());
                        i += 2;
                    }
                    PixelFormat::Rgb888 => {
                        out[i..i + 3].copy_from_slice(&[r, g, b]);
                        i += 3;
                    }
                    _ => return Err(Error::Unsupported),
                }
            }
        }
        let frame = Frame::packed(self.geometry, self.timestamp, self.sequence, &out[..needed])?;
        self.sequence = self.sequence.wrapping_add(1);
        self.timestamp = self.timestamp.add_micros(self.frame_micros);
        Ok(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pattern_is_deterministic_and_advances() {
        let g = Geometry::new(14, 4, PixelFormat::Rgb888).unwrap();
        let mut a = TestPattern::new(g, 10).unwrap();
        let mut b = TestPattern::new(g, 10).unwrap();
        let mut ba = [0u8; 14 * 4 * 3];
        let mut bb = [0u8; 14 * 4 * 3];
        let fa = a.grab(&mut ba).unwrap();
        assert_eq!(fa.sequence, 0);
        assert_eq!(fa.timestamp, Micros::ZERO);
        let fb = b.grab(&mut bb).unwrap();
        assert_eq!(fa.byte_len(), fb.byte_len());
        assert_eq!(ba, bb);
        // second frame: marker moved, timestamp advanced by 100 ms
        let f2 = a.grab(&mut ba).unwrap();
        assert_eq!(f2.sequence, 1);
        assert_eq!(f2.timestamp.as_millis(), 100);
        assert_ne!(ba, bb);
        // frame 0 (still in `bb`): row 0 is the marker, row 1 starts with bar 0 (0xC0)
        let row1 = 14 * 3;
        assert_eq!(&bb[row1..row1 + 3], &[0xC0, 0xC0, 0xC0]);
        assert_eq!(&bb[..3], &[0xFF, 0xFF, 0xFF], "marker row of frame 0");
        // frame 1 (in `ba`): the marker moved to row 1
        assert_eq!(&ba[row1..row1 + 3], &[0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn rejects_unsupported_and_small_buffers() {
        let g = Geometry::new(8, 8, PixelFormat::Jpeg).unwrap();
        assert_eq!(TestPattern::new(g, 10).err(), Some(Error::Unsupported));
        let g = Geometry::new(8, 8, PixelFormat::Gray8).unwrap();
        assert_eq!(TestPattern::new(g, 0).err(), Some(Error::InvalidGeometry));
        let mut p = TestPattern::new(g, 30).unwrap();
        let mut small = [0u8; 10];
        assert_eq!(
            p.grab(&mut small).err(),
            Some(Error::BufferTooSmall { needed: 64 })
        );
    }
}
