//! Pixel operations — scalar, bounds-checked, and the oracle for every faster
//! twin.
//!
//! The conversions and downscales a camera pipeline needs moved to
//! `rusty_esp_dsp::pixel` (D0, 2026-09-02) and are re-exported here at the
//! paths this crate always had, so a caller sees no change; the crop and
//! rotate helpers, which only this crate uses, stay. Colour conversion is
//! BT.601 full-range (the JFIF convention), fixed-point with an 8-bit
//! fraction — the same arithmetic on every platform.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::{Geometry, PixelFormat};

pub use rusty_esp_dsp::pixel::{
    downscale2x_gray8, downscale2x_rgb565, pack_rgb565, rgb565_to_rgb888, rgb888_to_rgb565,
    unpack_rgb565, yuv_to_rgb, yuyv_to_gray8, yuyv_to_rgb565, yuyv_to_rgb888,
};

fn expect_len(buf: &[u8], needed: usize) -> Result<()> {
    if buf.len() < needed {
        Err(Error::BufferTooSmall { needed })
    } else {
        Ok(())
    }
}

/// Crop a packed frame to the rectangle at (`x`, `y`) of size `w`×`h`.
pub fn crop(
    src: &[u8],
    geometry: Geometry,
    x: u32,
    y: u32,
    w: u32,
    h: u32,
    dst: &mut [u8],
) -> Result<Geometry> {
    let bpp = geometry
        .format
        .packed_bits_per_pixel()
        .ok_or(Error::Unsupported)? as usize
        / 8;
    if x.checked_add(w).is_none_or(|r| r > geometry.width)
        || y.checked_add(h).is_none_or(|b| b > geometry.height)
    {
        return Err(Error::InvalidGeometry);
    }
    let out = Geometry::new(w, h, geometry.format)?;
    expect_len(src, geometry.byte_len().ok_or(Error::Unsupported)?)?;
    let row_out = w as usize * bpp;
    expect_len(dst, row_out * h as usize)?;
    let stride = geometry.width as usize * bpp;
    for row in 0..h as usize {
        let s = (y as usize + row) * stride + x as usize * bpp;
        dst[row * row_out..(row + 1) * row_out].copy_from_slice(&src[s..s + row_out]);
    }
    Ok(out)
}

/// Rotate a Gray8 image 90° clockwise. Output is `height`×`width`.
pub fn rotate90_gray8(src: &[u8], width: u32, height: u32, dst: &mut [u8]) -> Result<Geometry> {
    let (w, h) = (width as usize, height as usize);
    expect_len(src, w * h)?;
    expect_len(dst, w * h)?;
    // A source ROW becomes a destination COLUMN. The original recomputed
    // `x * h` on every inner trip -- `x` IS the inner variable, so that
    // multiply could not be hoisted -- and bounds-checked both sides per
    // pixel. Walking the row forward against a destination cursor that steps
    // by `h` says the same thing with an add.
    if w != 0 && h != 0 {
        let dst = &mut dst[..w * h];
        let src = &src[..w * h];
        // A transpose cannot make both sides sequential, so TILE it and make
        // both sides local instead. Walking whole rows stores every pixel of
        // a row to a different destination line: the line is fetched, one
        // byte is written, and it is evicted long before the next row wants
        // it. An 8x8 tile touches 8 source lines and 8 destination lines for
        // its 64 pixels, and the destination run inside a tile is contiguous.
        // EIGHT. Sixteen measured +422% against a 0.0% null arm on an ESP32-S3
        // (2026-09-19) -- a cliff, not a slope, and worth re-deriving before
        // anyone widens it again.
        const T: usize = 8;
        let mut x0 = 0;
        while x0 < w {
            let xe = (x0 + T).min(w);
            let mut y0 = 0;
            while y0 < h {
                let ye = (y0 + T).min(h);
                for x in x0..xe {
                    // (x, y) -> (h - 1 - y, x) in an image of width h, so
                    // column `x` of the output is one contiguous run.
                    let col = &mut dst[x * h..x * h + h];
                    for y in y0..ye {
                        col[h - 1 - y] = src[y * w + x];
                    }
                }
                y0 += T;
            }
            x0 += T;
        }
    }
    Geometry::new(height, width, PixelFormat::Gray8)
}

/// Rotate a packed image 180° in place-compatible fashion (into `dst`).
pub fn rotate180(src: &[u8], bytes_per_pixel: usize, dst: &mut [u8]) -> Result<usize> {
    if bytes_per_pixel == 0 || src.len() % bytes_per_pixel != 0 {
        return Err(Error::InvalidGeometry);
    }
    expect_len(dst, src.len())?;
    let n = src.len() / bytes_per_pixel;
    // `(n - 1 - i) * bytes_per_pixel` is a descending cursor written as a
    // multiply: it steps down by exactly `bytes_per_pixel` every trip. A
    // REVERSED chunk walk says that directly -- no multiply, no per-pixel
    // bounds check -- and the fixed-width arms name a length the compiler can
    // see instead of a runtime one.
    //
    // NOT a measured win, and the record is worth keeping straight. A
    // same-build A/B against the original shape (both arms in one ESP32-S3
    // binary) read 137 726 -> 125 221 ps/px, -9.1%, on 2026-09-19. The same
    // A/B in the FIVE builds after that read the two arms identical to within
    // 8 ps. One probe said win, five said wash: the -9.1% was codegen that
    // did not survive two more crates entering the binary, not this rewrite.
    // It is kept because it is byte-identical, says what it means, and is no
    // worse -- not because it is faster.
    let dst = &mut dst[..src.len()];
    macro_rules! reversed {
        ($bpp:expr) => {{
            for (d, s) in dst.chunks_exact_mut($bpp).rev().zip(src.chunks_exact($bpp)) {
                d.copy_from_slice(s);
            }
        }};
    }
    match bytes_per_pixel {
        1 => reversed!(1),
        2 => reversed!(2),
        3 => reversed!(3),
        4 => reversed!(4),
        bpp => reversed!(bpp),
    }
    Ok(n)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn downscale_crop_rotate() {
        // 4x2 gray
        let src = [0u8, 4, 8, 12, 4, 8, 12, 16];
        let mut dst = [0u8; 2];
        let g = downscale2x_gray8(&src, 4, 2, &mut dst).unwrap();
        assert_eq!((g.width, g.height), (2, 1));
        assert_eq!(dst, [4, 12]);

        // 2x2 rgb565 of one colour averages to itself
        let c = pack_rgb565(200, 100, 50).to_le_bytes();
        let src: std::vec::Vec<u8> = c.iter().copied().cycle().take(8).collect();
        let mut dst = [0u8; 2];
        downscale2x_rgb565(&src, 2, 2, &mut dst).unwrap();
        assert_eq!(dst, c);

        // crop 1x2 out of a 3x2 gray8 image at x=1
        let src = [1u8, 2, 3, 4, 5, 6];
        let geo = Geometry::new(3, 2, PixelFormat::Gray8).unwrap();
        let mut dst = [0u8; 2];
        let g = crop(&src, geo, 1, 0, 1, 2, &mut dst).unwrap();
        assert_eq!((g.width, g.height), (1, 2));
        assert_eq!(dst, [2, 5]);
        assert_eq!(
            crop(&src, geo, 3, 0, 1, 1, &mut dst),
            Err(Error::InvalidGeometry)
        );

        // rotate 90 cw: 3x2 -> 2x3
        let mut r = [0u8; 6];
        let g = rotate90_gray8(&src, 3, 2, &mut r).unwrap();
        assert_eq!((g.width, g.height), (2, 3));
        assert_eq!(r, [4, 1, 5, 2, 6, 3]);

        let mut r = [0u8; 6];
        rotate180(&src, 1, &mut r).unwrap();
        assert_eq!(r, [6, 5, 4, 3, 2, 1]);
        let mut r2 = [0u8; 6];
        rotate180(&[1, 2, 3, 4, 5, 6], 2, &mut r2).unwrap();
        assert_eq!(r2, [5, 6, 3, 4, 1, 2]);
    }
}
