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
    // Two shapes were tried here on an ESP32-S3 (2026-09-19) and NEITHER beat
    // this one, so it stands unchanged:
    //   `bytes.windows(2).rposition(..)`  +35.7%  -- building a two-element
    //       slice per position costs more than the second load it saves;
    //   carrying the previous byte in a local, one load per position instead
    //       of two: 175 274 vs 175 258 ps/byte, i.e. FLAT. The second load is
    //       an L1 hit on a line already resident, so removing it buys nothing.
    // Reverted as "inside the noise", not as "measured worse".
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

/// The end of the **first** image in `bytes`: the offset just past its EOI
/// marker, found by walking the marker segments from SOI to the start of
/// scan and then the entropy-coded data, where `FF` is followed only by
/// `00` (a stuffed byte), a restart marker, or the marker that ends the
/// scan. A sensor in JPEG mode over DVP fills the rest of its frame after
/// the image — the OV3660 repeats the whole image in fixed blocks up to
/// VSYNC — so the last `FF D9` in a DMA buffer is not the frame's end;
/// this is. `None` when `bytes` does not start with SOI or the scan never
/// ends.
#[must_use]
pub fn find_first_eoi(bytes: &[u8]) -> Option<usize> {
    if bytes.len() < 4 || bytes[0] != 0xFF || bytes[1] != SOI {
        return None;
    }
    let mut i = 2usize;
    // the marker segments up to the first start of scan
    loop {
        if i + 4 > bytes.len() || bytes[i] != 0xFF {
            return None;
        }
        let marker = bytes[i + 1];
        match marker {
            0xFF => {
                // fill bytes before a marker
                i += 1;
                continue;
            }
            0xD8 | 0xD0..=0xD7 | 0x01 => {
                i += 2;
                continue;
            }
            EOI => return Some(i + 2),
            0xDA => {
                let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                i += 2 + len;
                break;
            }
            _ => {
                let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                i += 2 + len;
            }
        }
    }
    // the entropy-coded data: `FF 00` and `FF D0..D7` are data, anything
    // else after `FF` is a marker, and the first EOI ends the image
    while i + 1 < bytes.len() {
        // most of a picture is entropy bytes with no `FF` among them: step
        // over eight at a time while their maximum says none is `FF` (the
        // per-byte loop below would only have stepped over them one by one)
        i = skip_no_ff(bytes, i);
        if i + 1 >= bytes.len() {
            break;
        }
        if bytes[i] == 0xFF {
            match bytes[i + 1] {
                0x00 | 0xD0..=0xD7 | 0xFF => {}
                EOI => return Some(i + 2),
                0xDA => {
                    // another scan (progressive): skip its header and go on
                    if i + 4 > bytes.len() {
                        return None;
                    }
                    let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                    i += 2 + len;
                    continue;
                }
                _ => {
                    if i + 4 > bytes.len() {
                        return None;
                    }
                    let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                    i += 2 + len;
                    continue;
                }
            }
        }
        i += 1;
    }
    None
}

/// The first index at or after `i` that may hold `FF`: whole runs of eight
/// bytes without one are stepped over (their unsigned maximum is below
/// `FF`). The rest is left to the caller's per-byte loop.
pub(crate) fn skip_no_ff(bytes: &[u8], mut i: usize) -> usize {
    while let Some(w) = bytes.get(i..i + 8) {
        let m = w[0]
            .max(w[1])
            .max(w[2].max(w[3]))
            .max(w[4].max(w[5]).max(w[6].max(w[7])));
        if m == 0xFF {
            break;
        }
        i += 8;
    }
    i
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
    /// `find_first_eoi` as it was before the eight-byte skip: the oracle.
    fn find_first_eoi_bytewise(bytes: &[u8]) -> Option<usize> {
        if bytes.len() < 4 || bytes[0] != 0xFF || bytes[1] != SOI {
            return None;
        }
        let mut i = 2usize;
        loop {
            if i + 4 > bytes.len() || bytes[i] != 0xFF {
                return None;
            }
            let marker = bytes[i + 1];
            match marker {
                0xFF => {
                    i += 1;
                    continue;
                }
                0xD8 | 0xD0..=0xD7 | 0x01 => {
                    i += 2;
                    continue;
                }
                EOI => return Some(i + 2),
                0xDA => {
                    let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                    i += 2 + len;
                    break;
                }
                _ => {
                    let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                    i += 2 + len;
                }
            }
        }
        while i + 1 < bytes.len() {
            if bytes[i] == 0xFF {
                match bytes[i + 1] {
                    0x00 | 0xD0..=0xD7 | 0xFF => {}
                    EOI => return Some(i + 2),
                    _ => {
                        if i + 4 > bytes.len() {
                            return None;
                        }
                        let len = usize::from(u16::from_be_bytes([bytes[i + 2], bytes[i + 3]]));
                        i += 2 + len;
                        continue;
                    }
                }
            }
            i += 1;
        }
        None
    }

    #[test]
    fn the_skip_finds_the_same_end_as_the_per_byte_search() {
        let mut x = 0x9E37_79B9u32;
        let mut next = move || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        for case in 0..3000u32 {
            let mut p = std::vec![0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x06, 1, 2, 3, 4];
            p.extend_from_slice(&[0xFF, 0xDA, 0x00, 0x04, 1, 2]);
            let len = (next() % 700) as usize;
            for _ in 0..len {
                let r = next();
                match r % 61 {
                    0 => p.extend_from_slice(&[0xFF, 0x00]),
                    1 => p.extend_from_slice(&[0xFF, 0xD0 + (r >> 8) as u8 % 8]),
                    2 => p.extend_from_slice(&[0xFF, 0xFF]),
                    3 if case % 5 == 0 => p.extend_from_slice(&[0xFF, 0xC4, 0x00, 0x03, 9]),
                    _ => p.push((r >> 16) as u8 & 0xFE),
                }
            }
            p.extend_from_slice(&[0xFF, 0xD9]);
            p.extend((0..next() % 20).map(|k| k as u8));
            // every truncation too: the cutter calls it on prefixes
            for cut in [
                p.len(),
                p.len() / 2,
                p.len().saturating_sub(1),
                (next() as usize) % (p.len() + 1),
            ] {
                assert_eq!(
                    find_first_eoi(&p[..cut]),
                    find_first_eoi_bytewise(&p[..cut]),
                    "case {case} cut {cut}"
                );
            }
        }
    }

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

    #[test]
    fn first_eoi_stops_at_the_first_image() {
        // a minimal image: SOI, an APP0, a DQT-less SOS with two bytes of
        // scan holding a stuffed FF, then EOI; then the same again
        let one: &[u8] = &[
            0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x04, 0x4A, 0x46, 0xFF, 0xDA, 0x00, 0x02, 0x12, 0xFF,
            0x00, 0x34, 0xFF, 0xD9,
        ];
        let mut twice = one.to_vec();
        twice.extend_from_slice(&[0u8; 7]);
        twice.extend_from_slice(one);
        assert_eq!(find_first_eoi(one), Some(one.len()));
        assert_eq!(find_first_eoi(&twice), Some(one.len()));
        assert_eq!(find_eoi(&twice), Some(twice.len()));
        assert_eq!(find_first_eoi(&one[..10]), None);
        assert_eq!(find_first_eoi(b"not a jpeg"), None);
    }
}
