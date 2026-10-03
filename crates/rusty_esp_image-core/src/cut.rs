//! Cutting JPEG pictures out of a camera's byte stream.
//!
//! A DVP sensor in JPEG mode writes pictures into a DMA ring with padding
//! between them (the OV3660 fills its output window with 6,144-byte blocks;
//! see image's ledger, X5). A grab copies the bytes of the next picture out of
//! the ring: everything before a start-of-image (`FF D8`) is skipped and
//! counted, the picture is copied into the caller's slot, and it ends at the
//! first `FF D9` that [`find_first_eoi`] agrees is the picture's end of image
//! (an `FF D9` can sit inside a header segment, such as an embedded
//! thumbnail, and is not the end). A picture larger than the slot is counted
//! and skipped.
//!
//! The state survives between calls, so the ring can be drained in whatever
//! pieces the DMA hands over. [`reference`] is the per-byte loop this was
//! lifted from (`rusty_esp_image-esp`'s `DvpCamera::grab`, X5), kept as the
//! oracle: every change to [`Cutter::feed`] is checked against it for
//! identical output, consumed counts and counters over random streams in
//! random pieces.

use crate::jpeg::{find_first_eoi, skip_no_ff};

/// The first index at or after `i` holding `FF`, or the end: eight bytes at
/// a time while their maximum says none is, then byte by byte.
fn next_ff(input: &[u8], i: usize) -> usize {
    let j = skip_no_ff(input, i);
    input[j..]
        .iter()
        .position(|&b| b == 0xFF)
        .map_or(input.len(), |k| j + k)
}

/// Copy `src` into `dst` up to its first `FF` (not copied) or the shorter
/// length; the bytes copied. On an ESP32-S3 with `pie-s3` this is one pass
/// on the vector unit, `rusty_esp_dsp-esp`'s `copy_until_ff` (round 2, R6);
/// elsewhere the scan and the block copy W6 made it.
fn copy_run(src: &[u8], dst: &mut [u8]) -> usize {
    #[cfg(all(feature = "pie-s3", target_arch = "xtensa"))]
    {
        rusty_esp_dsp_esp::pie_s3::copy_until_ff(src, dst)
    }
    #[cfg(not(all(feature = "pie-s3", target_arch = "xtensa")))]
    {
        let m = src.len().min(dst.len());
        let k = next_ff(&src[..m], 0);
        dst[..k].copy_from_slice(&src[..k]);
        k
    }
}

/// The counters a cut keeps, added to as bytes go by.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CutCounts {
    /// Bytes skipped between pictures: padding, and the start-of-image's own
    /// second byte.
    pub skipped: u64,
    /// Pictures that did not fit the slot and were skipped.
    pub too_small: u64,
    /// `FF D9` pairs inside a picture that were not its end.
    pub false_eoi: u64,
}

/// Where a cut is between calls.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Cutter {
    in_frame: bool,
    prev_ff: bool,
    len: usize,
    /// `find_first_eoi`'s walk over the picture so far (W13).
    walk: Walk,
}

/// Where [`find_first_eoi`]'s walk over a growing picture has got to: the
/// same steps, taken as the bytes arrive. It waits wherever
/// `find_first_eoi` would answer `None` for want of bytes; `Done(p)` and
/// `Dead` are final, as `find_first_eoi`'s answer is for every longer
/// prefix once its walk has reached them. So `find_first_eoi(&b[..n]) ==
/// Some(n)` exactly when, with `n` bytes in, the walk is `Done(n)`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum Walk {
    /// No picture.
    #[default]
    Idle,
    /// In the marker segments, at the marker that starts at this offset.
    Header(usize),
    /// In the entropy-coded data, at this offset.
    Entropy(usize),
    /// The first end of image ends here.
    Done(usize),
    /// The header broke (a segment that does not start with `FF`).
    Dead,
}

impl Walk {
    /// Take every step `b` (the picture's first `b.len()` bytes) allows.
    /// `clean` is a range of `b` known to hold no `FF`: entropy steps over
    /// it are taken at once instead of byte by byte.
    fn advance(&mut self, b: &[u8], clean: (usize, usize)) {
        let n = b.len();
        loop {
            match *self {
                Walk::Idle | Walk::Done(_) | Walk::Dead => return,
                Walk::Header(i) => {
                    if i + 4 > n {
                        return;
                    }
                    if b[i] != 0xFF {
                        *self = Walk::Dead;
                        return;
                    }
                    let len = usize::from(u16::from_be_bytes([b[i + 2], b[i + 3]]));
                    *self = match b[i + 1] {
                        0xFF => Walk::Header(i + 1),
                        0xD8 | 0xD0..=0xD7 | 0x01 => Walk::Header(i + 2),
                        0xD9 => Walk::Done(i + 2),
                        0xDA => Walk::Entropy(i + 2 + len),
                        _ => Walk::Header(i + 2 + len),
                    };
                }
                Walk::Entropy(mut i) => {
                    // over a range with no `FF`, every step is `i += 1`
                    // while `i + 1 < n`
                    if i >= clean.0 && i < clean.1 {
                        i = i.max(clean.1.min(n.saturating_sub(1)));
                    }
                    if i + 1 >= n {
                        *self = Walk::Entropy(i);
                        return;
                    }
                    if b[i] == 0xFF {
                        match b[i + 1] {
                            0x00 | 0xD0..=0xD7 | 0xFF => {}
                            0xD9 => {
                                *self = Walk::Done(i + 2);
                                return;
                            }
                            _ => {
                                if i + 4 > n {
                                    *self = Walk::Entropy(i);
                                    return;
                                }
                                let len = usize::from(u16::from_be_bytes([b[i + 2], b[i + 3]]));
                                *self = Walk::Entropy(i + 2 + len);
                                continue;
                            }
                        }
                    }
                    *self = Walk::Entropy(i + 1);
                }
            }
        }
    }
}

impl Cutter {
    /// Between pictures, nothing copied.
    #[must_use]
    pub const fn new() -> Self {
        Cutter {
            in_frame: false,
            prev_ff: false,
            len: 0,
            walk: Walk::Idle,
        }
    }

    /// Back to between pictures (after the ring restarts).
    pub fn reset(&mut self) {
        *self = Cutter::new();
    }

    /// Bytes of the current picture copied into the slot so far.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// True when no picture has begun.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Take bytes from `input` into `out` (the same slot on every call of
    /// one grab). Returns how many bytes of `input` were used and, when a
    /// picture ended in them, its length in `out`; the bytes after it are
    /// left for the next call.
    pub fn feed(
        &mut self,
        input: &[u8],
        out: &mut [u8],
        counts: &mut CutCounts,
    ) -> (usize, Option<usize>) {
        let mut i = 0usize;
        while i < input.len() {
            if !self.in_frame {
                // W5: between pictures, a byte that is not `FF` after a byte
                // that was not `FF` only counts as skipped: run to the next
                // `FF` and count the run at once
                if !self.prev_ff {
                    let j = next_ff(input, i);
                    counts.skipped += (j - i) as u64;
                    i = j;
                    if i == input.len() {
                        break;
                    }
                }
                let b = input[i];
                i += 1;
                counts.skipped += 1;
                if self.prev_ff && b == 0xD8 {
                    self.in_frame = true;
                    out[0] = 0xFF;
                    out[1] = 0xD8;
                    self.len = 2;
                    self.prev_ff = false;
                    self.walk = Walk::Header(2);
                } else {
                    self.prev_ff = b == 0xFF;
                }
                continue;
            }
            if self.len >= out.len() {
                // the picture does not fit: skip to the next one
                i += 1;
                counts.too_small += 1;
                self.in_frame = false;
                self.len = 0;
                self.prev_ff = false;
                self.walk = Walk::Idle;
                continue;
            }
            // W6: inside a picture, bytes that are not `FF` after a byte that
            // was not `FF` are only copied: copy the run up to the next `FF`
            // (or the slot's end) at once
            if !self.prev_ff {
                let end = input.len().min(i + (out.len() - self.len));
                let from = self.len;
                let k = copy_run(&input[i..end], &mut out[from..]);
                if k > 0 {
                    let j = i + k;
                    self.len += k;
                    i = j;
                    // the walk over the run, which holds no `FF` (W13)
                    self.walk.advance(&out[..self.len], (from, self.len));
                    continue;
                }
            }
            let b = input[i];
            i += 1;
            out[self.len] = b;
            self.len += 1;
            if self.prev_ff && b == 0xD9 {
                // what `find_first_eoi(&out[..len]) == Some(len)` answers,
                // from the walk kept as the bytes came in (W13)
                self.walk.advance(&out[..self.len], (0, 0));
                if self.walk == Walk::Done(self.len) {
                    let n = self.len;
                    // the next grab starts between pictures
                    *self = Cutter::new();
                    return (i, Some(n));
                }
                counts.false_eoi += 1;
            }
            self.prev_ff = b == 0xFF;
        }
        (i, None)
    }
}

/// The per-byte cut, as `DvpCamera::grab` ran it: the oracle.
#[doc(hidden)]
pub fn reference(
    c: &mut Cutter,
    input: &[u8],
    out: &mut [u8],
    counts: &mut CutCounts,
) -> (usize, Option<usize>) {
    let mut consumed = 0usize;
    for &b in input {
        consumed += 1;
        if !c.in_frame {
            // between pictures: the padding until the next SOI
            counts.skipped += 1;
            if c.prev_ff && b == 0xD8 {
                c.in_frame = true;
                out[0] = 0xFF;
                out[1] = 0xD8;
                c.len = 2;
                c.prev_ff = false;
            } else {
                c.prev_ff = b == 0xFF;
            }
            continue;
        }
        if c.len >= out.len() {
            // the picture does not fit: skip to the next one
            counts.too_small += 1;
            c.in_frame = false;
            c.len = 0;
            c.prev_ff = false;
            continue;
        }
        out[c.len] = b;
        c.len += 1;
        if c.prev_ff && b == 0xD9 {
            if find_first_eoi(&out[..c.len]) == Some(c.len) {
                let n = c.len;
                // the next grab starts between pictures
                *c = Cutter::new();
                return (consumed, Some(n));
            }
            counts.false_eoi += 1;
        }
        c.prev_ff = b == 0xFF;
    }
    (consumed, None)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec;
    use std::vec::Vec;

    /// xorshift64*: deterministic noise for the property tests.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }
        fn below(&mut self, n: usize) -> usize {
            (self.next() % n as u64) as usize
        }
    }

    /// A small baseline-shaped JPEG: SOI, an APP0 segment (sometimes with
    /// an `FF D9` inside, as a thumbnail would), a DQT, SOS, entropy bytes
    /// with stuffed `FF 00` and restart markers, EOI.
    fn picture(r: &mut Lcg, entropy: usize, thumb_eoi: bool) -> Vec<u8> {
        let mut p = vec![0xFF, 0xD8];
        let app: Vec<u8> = if thumb_eoi {
            vec![b'J', b'F', b'I', b'F', 0, 0xFF, 0xD9, 0xFF, 0xD8, 7]
        } else {
            vec![b'J', b'F', b'I', b'F', 0, 1, 2]
        };
        p.extend_from_slice(&[0xFF, 0xE0]);
        p.extend_from_slice(&((app.len() + 2) as u16).to_be_bytes());
        p.extend_from_slice(&app);
        p.extend_from_slice(&[0xFF, 0xDB, 0x00, 0x05, 0, 1, 2]);
        p.extend_from_slice(&[0xFF, 0xDA, 0x00, 0x04, 1, 2]);
        for i in 0..entropy {
            match r.below(40) {
                0 => p.extend_from_slice(&[0xFF, 0x00]),
                1 if i % 7 == 0 => p.extend_from_slice(&[0xFF, 0xD0 + (i % 8) as u8]),
                _ => p.push((r.next() as u8) & 0xFE),
            }
        }
        p.extend_from_slice(&[0xFF, 0xD9]);
        p
    }

    /// Bytes that break the walk's assumptions: an EOI in the header, a
    /// segment without `FF`, fill bytes, truncated lengths, stray markers.
    fn hostile(r: &mut Lcg) -> Vec<u8> {
        let mut s = Vec::new();
        for _ in 0..1 + r.below(6) {
            s.extend_from_slice(&[0xFF, 0xD8]);
            for _ in 0..r.below(40) {
                let pick = r.below(12);
                match pick {
                    0 => s.extend_from_slice(&[0xFF, 0xD9]),
                    1 => s.extend_from_slice(&[0xFF, 0xFF, 0xFF]),
                    2 => s.extend_from_slice(&[0xFF, 0xDA, 0x00, (r.below(8)) as u8]),
                    3 => s.extend_from_slice(&[0xFF, 0xE1, 0x00, (r.below(12)) as u8]),
                    4 => s.extend_from_slice(&[0xFF, 0x00]),
                    5 => s.extend_from_slice(&[0xFF, 0xD0 + r.below(8) as u8]),
                    6 => s.extend_from_slice(&[0xFF, 0x01]),
                    7 => s.push(0xFF),
                    _ => {
                        for _ in 0..r.below(30) {
                            s.push((r.next() as u8) & 0xFE);
                        }
                    }
                }
            }
        }
        s
    }

    fn stream(r: &mut Lcg) -> Vec<u8> {
        let mut s = Vec::new();
        for _ in 0..1 + r.below(4) {
            // padding: zeros, stray FFs, a lone FF before D8 as data
            for _ in 0..r.below(300) {
                s.push(match r.below(10) {
                    0 => 0xFF,
                    1 => 0xD9,
                    _ => 0,
                });
            }
            let len = r.below(600);
            let thumb = r.below(3) == 0;
            s.extend_from_slice(&picture(r, len, thumb));
        }
        s
    }

    /// Run a whole stream through `feed` in random pieces, collecting every
    /// picture, the consumed totals and the counters.
    type FeedFn = fn(&mut Cutter, &[u8], &mut [u8], &mut CutCounts) -> (usize, Option<usize>);
    type Outcome = (Vec<Vec<u8>>, CutCounts, Vec<usize>);

    fn run(f: FeedFn, s: &[u8], slot: usize, r: &mut Lcg) -> Outcome {
        let mut c = Cutter::new();
        let mut counts = CutCounts::default();
        let mut out = vec![0u8; slot];
        let mut pics = Vec::new();
        let mut steps = Vec::new();
        let mut at = 0usize;
        while at < s.len() {
            let n = 1 + r.below(97);
            let piece = &s[at..(at + n).min(s.len())];
            let (used, done) = f(&mut c, piece, &mut out, &mut counts);
            steps.push(used);
            at += used;
            if let Some(len) = done {
                pics.push(out[..len].to_vec());
            }
        }
        (pics, counts, steps)
    }

    fn feed(c: &mut Cutter, i: &[u8], o: &mut [u8], k: &mut CutCounts) -> (usize, Option<usize>) {
        c.feed(i, o, k)
    }

    #[test]
    fn the_cut_matches_the_per_byte_oracle_on_hostile_streams() {
        for seed in 1..3000u64 {
            let mut r = Lcg(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
            let s = hostile(&mut r);
            let slot = [8usize, 40, 300, 4096][r.below(4)];
            let pieces = r.next();
            let a = run(reference, &s, slot, &mut Lcg(pieces | 1));
            let b = run(feed, &s, slot, &mut Lcg(pieces | 1));
            assert_eq!(a, b, "seed {seed}");
        }
    }

    #[test]
    fn the_cut_matches_the_per_byte_oracle_on_random_streams() {
        for seed in 1..400u64 {
            let mut r = Lcg(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
            let s = stream(&mut r);
            let slot = [64usize, 300, 4096][r.below(3)];
            let pieces = r.next();
            let a = run(reference, &s, slot, &mut Lcg(pieces | 1));
            let b = run(feed, &s, slot, &mut Lcg(pieces | 1));
            assert_eq!(a, b, "seed {seed}");
        }
    }

    #[test]
    fn a_picture_is_cut_whole_and_the_padding_counted() {
        let mut r = Lcg(7);
        let p = picture(&mut r, 50, true);
        let mut s = vec![0u8; 10];
        s.extend_from_slice(&p);
        s.extend_from_slice(&[0, 0, 0]);
        let mut c = Cutter::new();
        let mut k = CutCounts::default();
        let mut out = vec![0u8; 1024];
        let (used, done) = c.feed(&s, &mut out, &mut k);
        assert_eq!(done, Some(p.len()));
        assert_eq!(&out[..p.len()], &p[..]);
        assert_eq!(used, 10 + p.len());
        assert_eq!(k.skipped, 12, "ten zeros, the SOI's two bytes");
        assert_eq!(k.false_eoi, 1, "the thumbnail's FF D9");
    }
}
