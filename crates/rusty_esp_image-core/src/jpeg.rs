//! Reading a JPEG's header without decoding it.
//!
//! A camera sensor in JPEG mode hands over coded bytes and nothing else; the
//! geometry has to be read back out of the SOF segment to build a `Frame` or
//! a stream header. [`probe`] walks the marker segments up to the first
//! start-of-frame; [`find_eoi`] trims the padding a DMA transfer appends after
//! the end-of-image marker.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::{Geometry, PixelFormat};

/// What the header says about a JPEG.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JpegInfo {
    /// Width, height, `PixelFormat::Jpeg`.
    pub geometry: Geometry,
    /// True for a progressive (SOF2/6/10/14) scan structure.
    pub progressive: bool,
    /// 1 (grayscale), 3 (YCbCr/RGB) or 4 (CMYK).
    pub components: u8,
    /// Sample precision in bits; 8 for every camera sensor.
    pub precision: u8,
    /// Offset of the SOF marker's `0xFF`, useful for header rewriting.
    pub sof_offset: usize,
}

const SOI: u8 = 0xD8;
const EOI: u8 = 0xD9;
const SOS: u8 = 0xDA;

fn is_sof(marker: u8) -> bool {
    matches!(
        marker,
        0xC0 | 0xC1 | 0xC2 | 0xC3 | 0xC5 | 0xC6 | 0xC7 | 0xC9 | 0xCA | 0xCB | 0xCD | 0xCE | 0xCF
    )
}

fn is_progressive(marker: u8) -> bool {
    matches!(marker, 0xC2 | 0xC6 | 0xCA | 0xCE)
}

/// True when `bytes` start with the SOI marker.
#[must_use]
pub fn is_jpeg(bytes: &[u8]) -> bool {
    bytes.len() >= 2 && bytes[0] == 0xFF && bytes[1] == SOI
}

/// Parse the header up to the first SOF segment.
pub fn probe(bytes: &[u8]) -> Result<JpegInfo> {
    if !is_jpeg(bytes) {
        return Err(Error::InvalidFormat);
    }
    let mut i = 2usize;
    loop {
        // Every segment starts with 0xFF; fill bytes (repeated 0xFF) are allowed.
        let &ff = bytes.get(i).ok_or(Error::InvalidFormat)?;
        if ff != 0xFF {
            return Err(Error::InvalidFormat);
        }
        while bytes.get(i) == Some(&0xFF) {
            i += 1;
        }
        let &marker = bytes.get(i).ok_or(Error::InvalidFormat)?;
        i += 1;
        match marker {
            0x00 => return Err(Error::InvalidFormat), // stuffed byte outside scan data
            0x01 | 0xD0..=0xD7 => continue,           // standalone markers
            EOI | SOS => return Err(Error::InvalidFormat), // no frame header seen
            _ => {}
        }
        let len = read_u16(bytes, i)? as usize;
        if len < 2 {
            return Err(Error::InvalidFormat);
        }
        if is_sof(marker) {
            let seg = bytes.get(i + 2..i + len).ok_or(Error::InvalidFormat)?;
            if seg.len() < 6 {
                return Err(Error::InvalidFormat);
            }
            let precision = seg[0];
            let height = u32::from(u16::from_be_bytes([seg[1], seg[2]]));
            let width = u32::from(u16::from_be_bytes([seg[3], seg[4]]));
            let components = seg[5];
            if height == 0 {
                // Height defined later by a DNL marker; no camera does this.
                return Err(Error::Unsupported);
            }
            let geometry = Geometry::new(width, height, PixelFormat::Jpeg)?;
            return Ok(JpegInfo {
                geometry,
                progressive: is_progressive(marker),
                components,
                precision,
                sof_offset: i - 2,
            });
        }
        i += len;
    }
}

fn read_u16(bytes: &[u8], at: usize) -> Result<u16> {
    let hi = *bytes.get(at).ok_or(Error::InvalidFormat)?;
    let lo = *bytes.get(at + 1).ok_or(Error::InvalidFormat)?;
    Ok(u16::from_be_bytes([hi, lo]))
}

/// Length of the JPEG up to and including its EOI marker, scanning back
/// from the end so trailing DMA padding is skipped. `None` when no EOI is
/// found.
#[must_use]
pub fn find_eoi(bytes: &[u8]) -> Option<usize> {
    if bytes.len() < 2 {
        return None;
    }
    let mut i = bytes.len() - 1;
    while i >= 1 {
        if bytes[i] == EOI && bytes[i - 1] == 0xFF {
            return Some(i + 1);
        }
        i -= 1;
    }
    None
}

/// JPEG encoding on the chip: a raw frame into a caller-owned buffer through
/// `rusty_jpeg` (`no_std` + `alloc`, the 0.4 release made for this).
///
/// YUYV is coded as the sensor delivered it (`rusty_jpeg::YuyvImage`, no
/// colour conversion of our own); RGB888, BGR888, RGBA8888 and Gray8 go
/// through the encoder's own colour types. The output is a baseline JPEG
/// with the standard Huffman tables, which is what the RTP/JPEG payloader
/// and every browser expect. Feature `jpeg`.
#[cfg(feature = "jpeg")]
pub mod encode {
    use rusty_esp_core::error::{Error, Result};
    use rusty_esp_core::frame::{Frame, Geometry, PixelFormat, Planes};
    use rusty_jpeg::encode::{ColorType, Encoder, EncodingError, SliceWriter, YuyvImage};

    /// A buffer this large always holds the JPEG of a `geometry` frame at
    /// any quality: three bytes per pixel (a baseline JPEG of noise at
    /// quality 100 stays under that) plus room for the headers.
    #[must_use]
    pub fn max_bytes(geometry: &Geometry) -> usize {
        (geometry.width as usize)
            .saturating_mul(geometry.height as usize)
            .saturating_mul(3)
            .saturating_add(4096)
    }

    /// Encode packed `data` of `geometry` at `quality` (1–100) into `out`;
    /// returns the JPEG's length. `Unsupported` for a format with no packed
    /// pixels (planar, coded), `InvalidGeometry` when `data` is short or a
    /// side exceeds 65 535, `BufferTooSmall` (with [`max_bytes`] as the need)
    /// when `out` cannot hold it.
    pub fn encode_packed(
        geometry: Geometry,
        data: &[u8],
        quality: u8,
        out: &mut [u8],
    ) -> Result<usize> {
        let (w, h) = (
            u16::try_from(geometry.width).map_err(|_| Error::InvalidGeometry)?,
            u16::try_from(geometry.height).map_err(|_| Error::InvalidGeometry)?,
        );
        if w == 0 || h == 0 {
            return Err(Error::InvalidGeometry);
        }
        let needed = geometry.byte_len().ok_or(Error::Unsupported)?;
        if data.len() < needed {
            return Err(Error::InvalidGeometry);
        }
        let quality = quality.clamp(1, 100);
        let mut writer = SliceWriter::new(out);
        let result = match geometry.format {
            PixelFormat::Yuyv422 => {
                let image =
                    YuyvImage::new(data, usize::from(w) * 2, w, h).ok_or(Error::InvalidGeometry)?;
                Encoder::new(&mut writer, quality).encode_image(image)
            }
            PixelFormat::Rgb888 => {
                Encoder::new(&mut writer, quality).encode(data, w, h, ColorType::Rgb)
            }
            PixelFormat::Bgr888 => {
                Encoder::new(&mut writer, quality).encode(data, w, h, ColorType::Bgr)
            }
            PixelFormat::Rgba8888 => {
                Encoder::new(&mut writer, quality).encode(data, w, h, ColorType::Rgba)
            }
            PixelFormat::Gray8 => {
                Encoder::new(&mut writer, quality).encode(data, w, h, ColorType::Luma)
            }
            _ => return Err(Error::Unsupported),
        };
        match result {
            Ok(()) => Ok(writer.written()),
            Err(EncodingError::BufferTooSmall) => Err(Error::BufferTooSmall {
                needed: max_bytes(&geometry),
            }),
            Err(_) => Err(Error::InvalidFormat),
        }
    }

    /// [`encode_packed`] for a borrowed frame.
    pub fn encode_frame(frame: &Frame<'_>, quality: u8, out: &mut [u8]) -> Result<usize> {
        match frame.planes {
            Planes::Packed(data) => encode_packed(frame.geometry, data, quality, out),
            Planes::Planar { .. } => Err(Error::Unsupported),
        }
    }

    #[cfg(all(test, feature = "std"))]
    mod tests {
        use super::*;
        use crate::jpeg::{find_eoi, probe};
        use crate::source::{ImageSource, TestPattern};

        #[test]
        fn colour_bars_round_trip_through_the_house_decoder() {
            let g = Geometry::new(96, 64, PixelFormat::Rgb888).unwrap();
            let mut pattern = TestPattern::new(g, 10).unwrap();
            let mut rgb = vec![0u8; g.byte_len().unwrap()];
            let frame = pattern.grab(&mut rgb).unwrap();
            let mut out = vec![0u8; max_bytes(&g)];
            let n = encode_frame(&frame, 85, &mut out).unwrap();
            let info = probe(&out[..n]).unwrap();
            assert_eq!(info.geometry.width, 96);
            assert_eq!(info.geometry.height, 64);
            assert!(!info.progressive);
            assert_eq!(find_eoi(&out[..n]), Some(n));
            let mut d = rusty_jpeg::Decoder::new(&out[..n]);
            let pixels = d.decode().unwrap();
            let back = d.info().unwrap();
            assert_eq!((back.width, back.height), (96, 64));
            assert_eq!(pixels.len(), 96 * 64 * 3);
            let err: u64 = pixels
                .iter()
                .zip(&rgb)
                .map(|(&a, &b)| u64::from(a.abs_diff(b)))
                .sum();
            let mean = err / (96 * 64 * 3);
            // colour bars are all hard edges: ringing puts the mean near 7 at q85
            assert!(mean < 12, "mean abs error {mean}");
        }

        #[test]
        fn yuyv_is_coded_as_delivered_and_the_luma_survives() {
            let g = Geometry::new(64, 32, PixelFormat::Yuyv422).unwrap();
            let mut yuyv = vec![0u8; g.byte_len().unwrap()];
            for (i, px) in yuyv.chunks_exact_mut(4).enumerate() {
                let y = ((i % 32) * 8) as u8;
                px.copy_from_slice(&[y, 128, y, 128]);
            }
            let mut out = vec![0u8; max_bytes(&g)];
            let n = encode_packed(g, &yuyv, 90, &mut out).unwrap();
            assert_eq!(probe(&out[..n]).unwrap().geometry.width, 64);
            let mut d = rusty_jpeg::Decoder::new(&out[..n]);
            let pixels = d.decode().unwrap();
            let info = d.info().unwrap();
            assert_eq!((info.width, info.height), (64, 32));
            // gray in, gray out: the decoder's green channel is the luma
            let mut err = 0u64;
            let mut count = 0u64;
            for (px, y) in pixels
                .chunks_exact(3)
                .zip(yuyv.chunks_exact(2).map(|p| p[0]))
            {
                err += u64::from(px[1].abs_diff(y));
                count += 1;
            }
            assert!(err / count < 8, "mean luma error {}", err / count);
        }

        #[test]
        fn the_refusals_name_their_reason() {
            let g = Geometry::new(64, 32, PixelFormat::Rgb888).unwrap();
            let rgb = vec![90u8; g.byte_len().unwrap()];
            let mut small = [0u8; 100];
            assert_eq!(
                encode_packed(g, &rgb, 80, &mut small),
                Err(Error::BufferTooSmall {
                    needed: max_bytes(&g)
                })
            );
            let mut out = vec![0u8; max_bytes(&g)];
            assert_eq!(
                encode_packed(g, &rgb[..10], 80, &mut out),
                Err(Error::InvalidGeometry)
            );
            let planar = Geometry::new(64, 32, PixelFormat::Yuv420p).unwrap();
            assert_eq!(
                encode_packed(planar, &rgb, 80, &mut out),
                Err(Error::Unsupported)
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal header: SOI, an APP0 segment, then SOF of the given kind.
    fn header(sof: u8, width: u16, height: u16, components: u8) -> [u8; 4 + 6 + 2 + 2 + 6] {
        let mut h = [0u8; 20];
        h[0..2].copy_from_slice(&[0xFF, 0xD8]);
        h[2..4].copy_from_slice(&[0xFF, 0xE0]); // APP0
        h[4..6].copy_from_slice(&4u16.to_be_bytes()); // len 4: two payload bytes
        h[6..8].copy_from_slice(b"JF");
        h[8..10].copy_from_slice(&[0xFF, sof]);
        h[10..12].copy_from_slice(&8u16.to_be_bytes()); // len 8: 6 payload bytes
        h[12] = 8;
        h[13..15].copy_from_slice(&height.to_be_bytes());
        h[15..17].copy_from_slice(&width.to_be_bytes());
        h[17] = components;
        h[18..20].copy_from_slice(&[0xFF, 0xD9]);
        h
    }

    #[test]
    fn baseline_and_progressive_headers() {
        let b = header(0xC0, 320, 240, 3);
        let info = probe(&b).unwrap();
        assert_eq!(info.geometry.width, 320);
        assert_eq!(info.geometry.height, 240);
        assert_eq!(info.geometry.format, PixelFormat::Jpeg);
        assert!(!info.progressive);
        assert_eq!(info.components, 3);
        assert_eq!(info.precision, 8);
        assert_eq!(info.sof_offset, 8);
        let p = header(0xC2, 1600, 1200, 1);
        let info = probe(&p).unwrap();
        assert!(info.progressive);
        assert_eq!(info.components, 1);
        assert_eq!((info.geometry.width, info.geometry.height), (1600, 1200));
    }

    #[test]
    fn rejects_non_jpeg_truncated_and_headerless() {
        assert_eq!(probe(&[0x89, b'P', b'N', b'G']), Err(Error::InvalidFormat));
        assert_eq!(probe(&[0xFF, 0xD8]), Err(Error::InvalidFormat));
        let mut h = header(0xC0, 8, 8, 3);
        assert!(probe(&h[..12]).is_err());
        // SOS before SOF
        h[9] = 0xDA;
        assert_eq!(probe(&h), Err(Error::InvalidFormat));
        // DNL-style zero height
        let z = header(0xC0, 8, 0, 3);
        assert_eq!(probe(&z), Err(Error::Unsupported));
    }

    #[test]
    fn eoi_scan_skips_padding() {
        let mut buf = header(0xC0, 8, 8, 3).to_vec();
        let len = buf.len();
        buf.extend_from_slice(&[0, 0, 0, 0, 0xFF, 0xFF]);
        assert_eq!(find_eoi(&buf), Some(len));
        assert_eq!(find_eoi(&[0xFF, 0xD8, 0x00]), None);
        assert_eq!(find_eoi(&[]), None);
    }

    #[test]
    fn real_jpegs_from_the_house_encoder() {
        // rusty_jpeg on the host is the oracle: encode known geometries,
        // baseline and progressive, colour and grayscale; the probe must read
        // them back.
        for (w, h) in [(16u16, 8u16), (160, 120), (320, 240), (99, 33)] {
            for progressive in [false, true] {
                let rgb: std::vec::Vec<u8> = (0..(w as usize * h as usize * 3))
                    .map(|i| (i % 251) as u8)
                    .collect();
                let mut out = std::vec::Vec::new();
                let mut enc = rusty_jpeg::encode::Encoder::new(&mut out, 80);
                enc.set_progressive(progressive);
                enc.encode(&rgb, w, h, rusty_jpeg::encode::ColorType::Rgb)
                    .unwrap();
                let info = probe(&out).unwrap();
                assert_eq!(info.geometry.width, u32::from(w));
                assert_eq!(info.geometry.height, u32::from(h));
                assert_eq!(info.progressive, progressive, "{w}x{h}");
                assert_eq!(info.components, 3);
                assert_eq!(find_eoi(&out), Some(out.len()));

                let gray: std::vec::Vec<u8> = rgb.iter().step_by(3).copied().collect();
                let mut out = std::vec::Vec::new();
                let mut enc = rusty_jpeg::encode::Encoder::new(&mut out, 80);
                enc.set_progressive(progressive);
                enc.encode(&gray, w, h, rusty_jpeg::encode::ColorType::Luma)
                    .unwrap();
                let info = probe(&out).unwrap();
                assert_eq!(info.components, 1);
                assert_eq!(info.geometry.width, u32::from(w));
            }
        }
    }
}
