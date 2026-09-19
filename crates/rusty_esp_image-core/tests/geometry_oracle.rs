//! The gate the rotate/crop kernels did not have.
//!
//! `src/ops.rs` carried ONE test covering five kernels at 3x2 pixels. That is
//! enough to catch a transposed formula and nothing else: not a stride bug at
//! a realistic geometry, not an edge column, not an odd dimension. These are
//! PROPERTIES rather than recorded values, so they keep their force if a
//! kernel is rewritten, and they are checked at sizes where a stride mistake
//! has room to show.

use rusty_esp_image_core::ops::{rotate90_gray8, rotate180};

/// A deterministic image whose every byte is a function of its position, so a
/// misplaced pixel is identifiable rather than merely wrong.
fn img(w: u32, h: u32, bpp: usize) -> Vec<u8> {
    (0..(w as usize * h as usize * bpp))
        .map(|i| (i.wrapping_mul(97).wrapping_add(i / 13)) as u8)
        .collect()
}

/// Every geometry that has ever hidden a stride bug: square, wide, tall,
/// odd-by-even, prime edges, and the degenerate single row and column.
const SIZES: &[(u32, u32)] = &[
    (1, 1),
    (1, 7),
    (7, 1),
    (2, 2),
    (3, 2),
    (2, 3),
    (4, 4),
    (5, 3),
    (16, 16),
    (17, 13),
    (13, 17),
    (31, 29),
    (160, 120),
    (120, 160),
];

#[test]
fn rotate90_four_times_is_the_identity() {
    for &(w, h) in SIZES {
        let src = img(w, h, 1);
        let mut a = vec![0u8; src.len()];
        let mut b = vec![0u8; src.len()];
        // 90 degrees swaps the axes, so the dimensions alternate.
        let g1 = rotate90_gray8(&src, w, h, &mut a).unwrap();
        assert_eq!((g1.width, g1.height), (h, w), "{w}x{h} first turn");
        let g2 = rotate90_gray8(&a, h, w, &mut b).unwrap();
        assert_eq!((g2.width, g2.height), (w, h));
        let g3 = rotate90_gray8(&b, w, h, &mut a).unwrap();
        assert_eq!((g3.width, g3.height), (h, w));
        rotate90_gray8(&a, h, w, &mut b).unwrap();
        assert_eq!(b, src, "rotate90 x4 must be the identity at {w}x{h}");
    }
}

/// Two turns of 90 is one turn of 180 -- two kernels checking each other.
#[test]
fn two_quarter_turns_equal_one_half_turn() {
    for &(w, h) in SIZES {
        let src = img(w, h, 1);
        let mut q = vec![0u8; src.len()];
        let mut qq = vec![0u8; src.len()];
        rotate90_gray8(&src, w, h, &mut q).unwrap();
        rotate90_gray8(&q, h, w, &mut qq).unwrap();

        let mut half = vec![0u8; src.len()];
        rotate180(&src, 1, &mut half).unwrap();
        assert_eq!(qq, half, "90+90 != 180 at {w}x{h}");
    }
}

#[test]
fn rotate180_is_its_own_inverse_at_every_pixel_width() {
    for &bpp in &[1usize, 2, 3, 4] {
        for &(w, h) in SIZES {
            let src = img(w, h, bpp);
            let mut a = vec![0u8; src.len()];
            let mut b = vec![0u8; src.len()];
            let n = rotate180(&src, bpp, &mut a).unwrap();
            assert_eq!(n, w as usize * h as usize, "pixel count bpp={bpp}");
            rotate180(&a, bpp, &mut b).unwrap();
            assert_eq!(b, src, "rotate180 twice must be the identity, bpp={bpp}");
            // and it really did reverse: the first pixel is the last one
            assert_eq!(&a[..bpp], &src[src.len() - bpp..], "bpp={bpp} {w}x{h}");
        }
    }
}

#[test]
fn rotate90_puts_a_known_pixel_where_the_formula_says() {
    // (x, y) -> (h - 1 - y, x) in an image of width h.
    let (w, h) = (17u32, 13u32);
    let src = img(w, h, 1);
    let mut dst = vec![0u8; src.len()];
    rotate90_gray8(&src, w, h, &mut dst).unwrap();
    for y in 0..h as usize {
        for x in 0..w as usize {
            assert_eq!(
                dst[x * h as usize + (h as usize - 1 - y)],
                src[y * w as usize + x],
                "pixel ({x},{y})"
            );
        }
    }
}

#[test]
fn geometry_errors_are_unchanged() {
    let src = img(4, 4, 1);
    let mut small = vec![0u8; 4];
    assert!(rotate90_gray8(&src, 4, 4, &mut small).is_err(), "short dst");
    assert!(rotate90_gray8(&src[..4], 4, 4, &mut [0u8; 16]).is_err(), "short src");
    assert!(rotate180(&src, 0, &mut [0u8; 16]).is_err(), "zero bpp");
    assert!(rotate180(&src, 3, &mut [0u8; 16]).is_err(), "not a whole pixel");
    assert!(rotate180(&src, 1, &mut small).is_err(), "short dst");
}

#[test]
fn find_eoi_agrees_with_a_naive_backward_scan() {
    fn naive(bytes: &[u8]) -> Option<usize> {
        if bytes.len() < 2 {
            return None;
        }
        let mut i = bytes.len() - 1;
        while i >= 1 {
            if bytes[i] == 0xD9 && bytes[i - 1] == 0xFF {
                return Some(i + 1);
            }
            i -= 1;
        }
        None
    }
    // empty, too short, no marker, marker at every position, trailing padding,
    // a split marker (0xFF at the end), and a run of 0xFF before the 0xD9.
    let mut cases: Vec<Vec<u8>> = vec![
        vec![],
        vec![0xFF],
        vec![0xD9],
        vec![0xFF, 0xD9],
        vec![0xD9, 0xFF],
        vec![0u8; 64],
        vec![0xFF; 64],
    ];
    for n in 2usize..24 {
        for p in 0..n - 1 {
            let mut v = vec![0x00u8; n];
            v[p] = 0xFF;
            v[p + 1] = 0xD9;
            cases.push(v.clone());
            v.push(0x00); // trailing DMA padding after the marker
            v.push(0xFF);
            cases.push(v);
        }
    }
    let mut lcg: u32 = 0x1234_5678;
    for _ in 0..2000 {
        let n = (lcg >> 20) as usize % 40;
        let v: Vec<u8> = (0..n)
            .map(|_| {
                lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                // heavily biased toward the marker bytes
                match lcg >> 29 {
                    0..=2 => 0xFF,
                    3..=5 => 0xD9,
                    _ => (lcg >> 8) as u8,
                }
            })
            .collect();
        cases.push(v);
    }
    for v in cases {
        assert_eq!(
            rusty_esp_image_core::jpeg::find_eoi(&v),
            naive(&v),
            "find_eoi disagreed on {v:02x?}"
        );
    }
}

/// `TestPattern::grab` against the per-pixel formula it replaced.
///
/// The rewrite is an ALGEBRAIC claim -- that `colour_at(x, y)` depends on `y`
/// only through `y == marker_y`, so a frame is two distinct rows -- and a
/// single recorded frame cannot test a claim like that. This drives every
/// format, a spread of geometries, and enough consecutive frames that the
/// marker sweeps through row 0 and past the last row, which is where the
/// "build the bars row somewhere that isn't the marker" choice can go wrong.
#[test]
fn testpattern_matches_the_per_pixel_formula() {
    use rusty_esp_core::frame::{Geometry, PixelFormat};
    use rusty_esp_image_core::ops::pack_rgb565;
    use rusty_esp_image_core::source::{ImageSource, TestPattern};

    const BARS: [[u8; 3]; 7] = [
        [0xC0, 0xC0, 0xC0],
        [0xC0, 0xC0, 0x00],
        [0x00, 0xC0, 0xC0],
        [0x00, 0xC0, 0x00],
        [0xC0, 0x00, 0xC0],
        [0xC0, 0x00, 0x00],
        [0x00, 0x00, 0xC0],
    ];

    /// The body of `grab` as it was written, pixel by pixel.
    fn as_written(w: u32, h: u32, fmt: PixelFormat, sequence: u32, out: &mut Vec<u8>) {
        out.clear();
        let marker_y = sequence % h.max(1);
        for y in 0..h {
            for x in 0..w {
                let [r, g, b] = if y == marker_y {
                    [0xFF, 0xFF, 0xFF]
                } else {
                    let bar = ((u64::from(x) * 7) / u64::from(w.max(1))) as usize;
                    BARS[bar.min(6)]
                };
                match fmt {
                    PixelFormat::Gray8 => out.push(
                        ((77 * u32::from(r) + 150 * u32::from(g) + 29 * u32::from(b)) >> 8) as u8,
                    ),
                    PixelFormat::Rgb565 => {
                        out.extend_from_slice(&pack_rgb565(r, g, b).to_le_bytes());
                    }
                    _ => out.extend_from_slice(&[r, g, b]),
                }
            }
        }
    }

    for fmt in [PixelFormat::Gray8, PixelFormat::Rgb565, PixelFormat::Rgb888] {
        for &(w, h) in &[
            (1u32, 1u32),
            (1, 5),
            (5, 1),
            (7, 7),
            (8, 3),
            (14, 4),
            (13, 9),
            (160, 120),
        ] {
            let g = Geometry::new(w, h, fmt).unwrap();
            let mut tp = TestPattern::new(g, 30).unwrap();
            let need = g.byte_len().unwrap();
            let mut got = vec![0u8; need];
            let mut want = Vec::with_capacity(need);
            // More frames than rows, so the marker crosses row 0 and wraps.
            for seq in 0..(h + 3) {
                let f = tp.grab(&mut got).unwrap();
                assert_eq!(f.sequence, seq, "sequence {w}x{h} {fmt:?}");
                as_written(w, h, fmt, seq, &mut want);
                assert_eq!(
                    &got[..need],
                    &want[..],
                    "grab != per-pixel formula at {w}x{h} {fmt:?} seq={seq}"
                );
            }
        }
    }
}
