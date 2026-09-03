//! The robustness gate: the JPEG probe takes whatever a sensor or a socket
//! hands it and answers or errs; it never panics. Random bytes from an LCG
//! with a bias toward JPEG marker bytes, and mutations of a minimal header,
//! under `catch_unwind` so a failure names the function and prints the input.

use std::panic::{AssertUnwindSafe, catch_unwind};

use rusty_esp_image_core::jpeg::{find_eoi, is_jpeg, probe};

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }

    fn bytes(&mut self, max_len: usize) -> Vec<u8> {
        let n = self.below(max_len + 1);
        (0..n).map(|_| (self.next() >> 56) as u8).collect()
    }

    fn markers(&mut self, max_len: usize) -> Vec<u8> {
        let mut v = self.bytes(max_len);
        for b in v.iter_mut() {
            match self.below(6) {
                0 => *b = 0xFF,
                1 => *b = 0xD8,
                2 => *b = 0xC0 + self.below(16) as u8,
                3 => *b = 0x00,
                _ => {}
            }
        }
        v
    }

    fn mutate(&mut self, base: &[u8]) -> Vec<u8> {
        let mut v = base.to_vec();
        match self.below(5) {
            0 if !v.is_empty() => {
                let i = self.below(v.len());
                v[i] ^= 1 << self.below(8);
            }
            1 if !v.is_empty() => {
                let i = self.below(v.len());
                v[i] = (self.next() >> 56) as u8;
            }
            2 => v.truncate(self.below(v.len() + 1)),
            3 => {
                let extra = self.markers(32);
                v.extend_from_slice(&extra);
            }
            _ if !v.is_empty() => {
                let i = self.below(v.len());
                v.remove(i);
            }
            _ => {}
        }
        v
    }
}

fn check<R>(name: &str, input: &[u8], f: impl FnOnce() -> R) {
    if catch_unwind(AssertUnwindSafe(f)).is_err() {
        let hex: String = input.iter().take(256).map(|b| format!("{b:02x}")).collect();
        panic!("{name} panicked on {} bytes: {hex}", input.len());
    }
}

/// SOI, APP0 (JFIF), DQT (one table), SOF0 (8×8 4:2:0), DHT stub, SOS, EOI.
fn minimal_jpeg() -> Vec<u8> {
    let mut v = vec![0xFF, 0xD8];
    v.extend_from_slice(&[0xFF, 0xE0, 0x00, 0x10]);
    v.extend_from_slice(b"JFIF\0");
    v.extend_from_slice(&[1, 1, 0, 0, 1, 0, 1, 0, 0]);
    v.extend_from_slice(&[0xFF, 0xDB, 0x00, 0x43, 0x00]);
    v.extend_from_slice(&[16u8; 64]);
    v.extend_from_slice(&[
        0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x08, 0x00, 0x08, 0x03, 0x01, 0x22, 0x00, 0x02, 0x11,
        0x00, 0x03, 0x11, 0x00,
    ]);
    v.extend_from_slice(&[0xFF, 0xC4, 0x00, 0x03, 0x00]);
    v.extend_from_slice(&[
        0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3F, 0x00,
    ]);
    v.extend_from_slice(&[0x12, 0x34, 0x56, 0x78]);
    v.extend_from_slice(&[0xFF, 0xD9]);
    v
}

#[test]
fn jpeg_probe_never_panics() {
    let mut rng = Lcg(0x1AE6_0001);
    let base = minimal_jpeg();
    assert!(is_jpeg(&base));
    assert_eq!(find_eoi(&base), Some(base.len()));
    for i in 0..40_000 {
        let input = match i % 3 {
            0 => rng.markers(300),
            _ => rng.mutate(&base),
        };
        check("jpeg::probe", &input, || probe(&input).map(|_| ()));
        check("jpeg::find_eoi", &input, || find_eoi(&input));
        check("jpeg::is_jpeg", &input, || is_jpeg(&input));
    }
}
