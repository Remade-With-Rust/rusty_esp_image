#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
//! `rusty_esp_image-core` — the pure heart of `rusty_esp_image`.
//!
//! esp32-camera and Espressif's JPEG components, remade: what a camera on an
//! ESP32 needs *above* the DMA engine, with no driver in sight.
//!
//! | Module | Holds |
//! |---|---|
//! | [`source`] | [`ImageSource`] — capture **into caller memory**; a `TestPattern` source for host and smoke tests |
//! | [`pool`] | [`FramePool`] — N fixed slots over one caller buffer: the DMA ring in user clothes |
//! | [`jpeg`] | [`jpeg::probe`] — geometry, precision and progressive flag from a JPEG header, no decode; `find_eoi` to trim DMA over-read |
//! | [`ops`] | pixel kernels (RGB565 ↔ RGB888, YUYV → RGB/gray, 2× downscale, crop, rotate) — scalar, tested; the oracles for any later PIE twin |
//! | [`sensor`] | sensor identity and register tables **as data** (`SensorDesc`, `Register`, the OV2640 / OV5640 tables derived from esp32-camera) |
//! | [`sccb`] | [`Sccb`] — the camera control bus (I²C without repeated start) over `embedded-hal` 1.0 |
//!
//! Rules (from the package plan): every buffer is caller-owned; nothing here
//! allocates on a hot path; `forbid(unsafe)`; the scalar path is the oracle.
//! The DVP / MIPI capture engines and the hardware JPEG codecs live in
//! `rusty_esp_image-esp`; the software JPEG codec is `rusty_jpeg`, behind a
//! feature once its encoder is `no_std`.

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod jpeg;
pub mod ops;
pub mod pool;
pub mod sccb;
pub mod sensor;
pub mod source;

pub use pool::{FramePool, SlotId};
pub use rusty_esp_core as esp_core;
pub use sccb::{RegWidth, Sccb};
pub use sensor::{Mode, Register, SensorDesc, SensorId};
pub use source::{ImageSource, TestPattern};

/// The names a sketch or firmware wants in scope.
pub mod prelude {
    pub use crate::jpeg::{JpegInfo, probe};
    pub use crate::pool::{FramePool, SlotId};
    pub use crate::sccb::{RegWidth, Sccb};
    pub use crate::sensor::{Mode, Register, SensorDesc, SensorId};
    pub use crate::source::{ImageSource, TestPattern};
    pub use rusty_esp_core::prelude::*;
}

/// Crate version, for capability manifests and logs.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
