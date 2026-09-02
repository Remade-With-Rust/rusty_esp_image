#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]
//! `rusty_esp_image-esp` — chip backends for `rusty_esp_image`.
//!
//! This is the **wrap** crate of the package: where the silicon must be
//! touched, it calls ESP-IDF (Track A, `esp-idf`) or the esp-rs HAL (Track B,
//! `esp-hal`) and exposes the core crate's `ImageSource` over it.
//!
//! Track A's [`idf::IdfCamera`] binds Espressif's `esp32-camera` component —
//! the honest label from the plan: *application in Rust, driver wrapped*. Its
//! FFI calls are the only `unsafe` in the package, each fenced with the
//! invariant it relies on.

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(all(feature = "esp-hal", feature = "esp-idf"))]
compile_error!("enable exactly one track: `esp-hal` (no_std) or `esp-idf` (std)");

pub use rusty_esp_image_core as core;

/// Which track this build of the backend crate was compiled for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Track {
    /// No chip backend compiled in: host build, traits only.
    Host,
    /// Track B — bare metal, esp-hal + Embassy.
    EspHal,
    /// Track A — std on ESP-IDF.
    EspIdf,
}

/// The track this crate was built with.
pub const TRACK: Track = if cfg!(feature = "esp-hal") {
    Track::EspHal
} else if cfg!(feature = "esp-idf") {
    Track::EspIdf
} else {
    Track::Host
};

/// The DVP/SCCB pins of a camera board. `-1` means "not connected".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CameraPins {
    /// Power-down.
    pub pwdn: i32,
    /// Reset.
    pub reset: i32,
    /// Master clock out.
    pub xclk: i32,
    /// SCCB data.
    pub sccb_sda: i32,
    /// SCCB clock.
    pub sccb_scl: i32,
    /// Data lines, `d[7]` is the MSB (Y9 on OmniVision naming).
    pub d: [i32; 8],
    /// Vertical sync.
    pub vsync: i32,
    /// Horizontal reference.
    pub href: i32,
    /// Pixel clock.
    pub pclk: i32,
}

/// Seeed Studio XIAO ESP32-S3 Sense (OV2640 on the Sense expansion board).
pub const XIAO_ESP32S3_SENSE: CameraPins = CameraPins {
    pwdn: -1,
    reset: -1,
    xclk: 10,
    sccb_sda: 40,
    sccb_scl: 39,
    // Y2..Y9 = d[0]..d[7]
    d: [15, 17, 18, 16, 14, 12, 11, 48],
    vsync: 38,
    href: 47,
    pclk: 13,
};

/// AI-Thinker ESP32-CAM (OV2640).
pub const AI_THINKER_ESP32_CAM: CameraPins = CameraPins {
    pwdn: 32,
    reset: -1,
    xclk: 0,
    sccb_sda: 26,
    sccb_scl: 27,
    d: [5, 18, 19, 21, 36, 39, 34, 35],
    vsync: 25,
    href: 23,
    pclk: 22,
};

#[cfg(feature = "esp-idf")]
pub mod idf;

#[cfg(feature = "esp-hal")]
pub mod hal {
    //! Track B backends. The DVP engine over `esp_hal::lcd_cam::cam` lands here.
}
