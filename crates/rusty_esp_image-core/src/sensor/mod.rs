//! Sensor identity and register tables — **as data**.
//!
//! esp32-camera carries one C driver per sensor. Janus carries the *facts*
//! those drivers encode — product ids, bus addresses, register widths,
//! maximum geometry, and the register sequences that configure the part —
//! and leaves the state machine that applies them to `rusty_esp_image-esp`
//! (I1), where a real bus and a real clock exist.
//!
//! The OV2640 and OV5640 tables are derived mechanically from esp32-camera
//! (Apache-2.0, attributed in `LICENSE-THIRD-PARTY.md`): macro names resolved,
//! terminators dropped, nothing else changed.

pub mod ov2640;
pub mod ov5640;

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::{Geometry, PixelFormat};

use crate::sccb::RegWidth;

/// One register write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Register {
    /// Register address (8 or 16 bits wide depending on the sensor).
    pub addr: u16,
    /// Value.
    pub value: u8,
}

impl Register {
    /// Build a register write.
    #[must_use]
    pub const fn new(addr: u16, value: u8) -> Self {
        Register { addr, value }
    }
}

/// One step of a register sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegOp {
    /// Write `value` to `addr`.
    Write {
        /// Register address.
        addr: u16,
        /// Value.
        value: u8,
    },
    /// Wait this many milliseconds before the next step.
    DelayMs(u16),
}

/// The sensors esp32-camera knows, with their product ids.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum SensorId {
    /// OmniVision OV9650.
    Ov9650,
    /// OmniVision OV7725.
    Ov7725,
    /// OmniVision OV2640 — the ESP32-CAM and XIAO S3 Sense sensor.
    Ov2640,
    /// OmniVision OV3660.
    Ov3660,
    /// OmniVision OV5640.
    Ov5640,
    /// OmniVision OV7670.
    Ov7670,
    /// Novatek NT99141.
    Nt99141,
    /// GalaxyCore GC2145.
    Gc2145,
    /// GalaxyCore GC032A.
    Gc032a,
    /// GalaxyCore GC0308.
    Gc0308,
    /// BYD BF3005.
    Bf3005,
    /// BYD BF20A6.
    Bf20a6,
    /// SmartSens SC101IOT.
    Sc101iot,
    /// SmartSens SC030IOT.
    Sc030iot,
    /// SmartSens SC031GS.
    Sc031gs,
    /// Arducam Mega CCM.
    MegaCcm,
    /// Himax HM1055.
    Hm1055,
    /// Himax HM0360.
    Hm0360,
}

impl SensorId {
    /// The product id as esp32-camera reads it (one or two bytes; see
    /// [`SensorDesc::pid_len`]).
    #[must_use]
    pub const fn pid(self) -> u16 {
        match self {
            SensorId::Ov9650 => 0x96,
            SensorId::Ov7725 => 0x77,
            SensorId::Ov2640 => 0x26,
            SensorId::Ov3660 => 0x3660,
            SensorId::Ov5640 => 0x5640,
            SensorId::Ov7670 => 0x76,
            SensorId::Nt99141 => 0x1410,
            SensorId::Gc2145 => 0x2145,
            SensorId::Gc032a => 0x232A,
            SensorId::Gc0308 => 0x9B,
            SensorId::Bf3005 => 0x30,
            SensorId::Bf20a6 => 0x20A6,
            SensorId::Sc101iot => 0xDA4A,
            SensorId::Sc030iot => 0x9A46,
            SensorId::Sc031gs => 0x0031,
            SensorId::MegaCcm => 0x039E,
            SensorId::Hm1055 => 0x0955,
            SensorId::Hm0360 => 0x0360,
        }
    }

    /// Look a sensor up by product id.
    #[must_use]
    pub fn from_pid(pid: u16) -> Option<SensorId> {
        ALL_IDS.iter().copied().find(|s| s.pid() == pid)
    }
}

const ALL_IDS: &[SensorId] = &[
    SensorId::Ov9650,
    SensorId::Ov7725,
    SensorId::Ov2640,
    SensorId::Ov3660,
    SensorId::Ov5640,
    SensorId::Ov7670,
    SensorId::Nt99141,
    SensorId::Gc2145,
    SensorId::Gc032a,
    SensorId::Gc0308,
    SensorId::Bf3005,
    SensorId::Bf20a6,
    SensorId::Sc101iot,
    SensorId::Sc030iot,
    SensorId::Sc031gs,
    SensorId::MegaCcm,
    SensorId::Hm1055,
    SensorId::Hm0360,
];

/// What a driver needs to find and address a sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorDesc {
    /// Which part.
    pub id: SensorId,
    /// Marketing name.
    pub name: &'static str,
    /// 7-bit SCCB address.
    pub sccb_addr: u8,
    /// Register address width.
    pub reg_width: RegWidth,
    /// Address of the (first) product-id register.
    pub pid_reg: u16,
    /// How many consecutive bytes form the product id (1 or 2).
    pub pid_len: u8,
    /// Expected product id.
    pub pid: u16,
    /// Largest frame the part produces.
    pub max_width: u32,
    /// Largest frame the part produces.
    pub max_height: u32,
}

impl SensorDesc {
    /// The maximum geometry in `format`.
    pub fn max_geometry(&self, format: PixelFormat) -> Result<Geometry> {
        Geometry::new(self.max_width, self.max_height, format)
    }
}

/// The sensors with a complete descriptor today: the OmniVision parts whose
/// id registers and bus addresses are documented in esp32-camera. Others get
/// theirs when their register tables land.
pub const SENSORS: &[SensorDesc] = &[
    SensorDesc {
        id: SensorId::Ov2640,
        name: "OV2640",
        sccb_addr: ov2640::SCCB_ADDR,
        reg_width: RegWidth::U8,
        // PIDH in the sensor bank; PIDL (0x0B) is 0x42.
        pid_reg: 0x0A,
        pid_len: 1,
        pid: SensorId::Ov2640.pid(),
        max_width: 1600,
        max_height: 1200,
    },
    SensorDesc {
        id: SensorId::Ov5640,
        name: "OV5640",
        sccb_addr: ov5640::SCCB_ADDR,
        reg_width: RegWidth::U16,
        pid_reg: 0x300A,
        pid_len: 2,
        pid: SensorId::Ov5640.pid(),
        max_width: 2560,
        max_height: 1920,
    },
    SensorDesc {
        id: SensorId::Ov3660,
        name: "OV3660",
        sccb_addr: 0x3C,
        reg_width: RegWidth::U16,
        pid_reg: 0x300A,
        pid_len: 2,
        pid: SensorId::Ov3660.pid(),
        max_width: 2048,
        max_height: 1536,
    },
    SensorDesc {
        id: SensorId::Ov7670,
        name: "OV7670",
        sccb_addr: 0x21,
        reg_width: RegWidth::U8,
        pid_reg: 0x0A,
        pid_len: 1,
        pid: SensorId::Ov7670.pid(),
        max_width: 640,
        max_height: 480,
    },
];

/// Find the descriptor for a sensor.
#[must_use]
pub fn describe(id: SensorId) -> Option<&'static SensorDesc> {
    SENSORS.iter().find(|d| d.id == id)
}

/// The standard frame sizes esp32-camera names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum FrameSize {
    /// 96×96.
    S96x96,
    /// 160×120.
    Qqvga,
    /// 128×128.
    S128x128,
    /// 176×144.
    Qcif,
    /// 240×176.
    Hqvga,
    /// 240×240.
    S240x240,
    /// 320×240.
    Qvga,
    /// 320×320.
    S320x320,
    /// 400×296.
    Cif,
    /// 480×320.
    Hvga,
    /// 640×480.
    Vga,
    /// 800×600.
    Svga,
    /// 1024×768.
    Xga,
    /// 1280×720.
    Hd,
    /// 1280×1024.
    Sxga,
    /// 1600×1200.
    Uxga,
    /// 1920×1080.
    Fhd,
    /// 2048×1536.
    Qxga,
    /// 2560×1440.
    Qhd,
    /// 2560×1920.
    Qsxga,
    /// 2592×1944.
    S5mp,
}

impl FrameSize {
    /// Width and height.
    #[must_use]
    pub const fn dimensions(self) -> (u32, u32) {
        match self {
            FrameSize::S96x96 => (96, 96),
            FrameSize::Qqvga => (160, 120),
            FrameSize::S128x128 => (128, 128),
            FrameSize::Qcif => (176, 144),
            FrameSize::Hqvga => (240, 176),
            FrameSize::S240x240 => (240, 240),
            FrameSize::Qvga => (320, 240),
            FrameSize::S320x320 => (320, 320),
            FrameSize::Cif => (400, 296),
            FrameSize::Hvga => (480, 320),
            FrameSize::Vga => (640, 480),
            FrameSize::Svga => (800, 600),
            FrameSize::Xga => (1024, 768),
            FrameSize::Hd => (1280, 720),
            FrameSize::Sxga => (1280, 1024),
            FrameSize::Uxga => (1600, 1200),
            FrameSize::Fhd => (1920, 1080),
            FrameSize::Qxga => (2048, 1536),
            FrameSize::Qhd => (2560, 1440),
            FrameSize::Qsxga => (2560, 1920),
            FrameSize::S5mp => (2592, 1944),
        }
    }

    /// The geometry in `format`.
    pub fn geometry(self, format: PixelFormat) -> Result<Geometry> {
        let (w, h) = self.dimensions();
        Geometry::new(w, h, format)
    }
}

/// What a firmware asks a sensor for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mode {
    /// Frame geometry and pixel format.
    pub geometry: Geometry,
    /// Target frame rate; 0 leaves the sensor's default.
    pub fps: u8,
    /// JPEG quality 1–63 (lower is better) when the format is JPEG.
    pub jpeg_quality: u8,
    /// Vertical flip.
    pub flip: bool,
    /// Horizontal mirror.
    pub mirror: bool,
}

impl Mode {
    /// A JPEG mode at `size`, quality 12 (the esp32-camera default).
    pub fn jpeg(size: FrameSize) -> Result<Self> {
        Ok(Mode {
            geometry: size.geometry(PixelFormat::Jpeg)?,
            fps: 0,
            jpeg_quality: 12,
            flip: false,
            mirror: false,
        })
    }

    /// An uncompressed mode at `size`: what a pipeline that reads pixels
    /// asks for.
    ///
    /// `jpeg()` was the only constructor, which is a large part of why every
    /// pixel kernel in this family had no production caller (see the
    /// reachability census, `rusty_esp_dsp` ledger R1): a camera that can
    /// only be asked for JPEG produces bytes nobody converts, downscales or
    /// filters. RGB565, YUYV422 and Gray8 are what the OV2640/OV5640 emit
    /// besides JPEG; anything else is rejected by `Geometry`.
    pub fn raw(size: FrameSize, format: PixelFormat) -> Result<Self> {
        match format {
            PixelFormat::Rgb565 | PixelFormat::Yuyv422 | PixelFormat::Gray8 => {}
            _ => return Err(Error::Unsupported),
        }
        Ok(Mode {
            geometry: size.geometry(format)?,
            fps: 0,
            // Not read for an uncompressed format; the driver ignores it.
            jpeg_quality: 12,
            flip: false,
            mirror: false,
        })
    }

    /// Check the mode against a sensor's limits.
    pub fn check(&self, desc: &SensorDesc) -> Result<()> {
        if self.geometry.width > desc.max_width || self.geometry.height > desc.max_height {
            return Err(Error::Unsupported);
        }
        if self.geometry.format == PixelFormat::Jpeg && !(1..=63).contains(&self.jpeg_quality) {
            return Err(Error::InvalidFormat);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pids_round_trip_and_descriptors_agree() {
        for id in ALL_IDS {
            assert_eq!(SensorId::from_pid(id.pid()), Some(*id));
        }
        for d in SENSORS {
            assert_eq!(d.pid, d.id.pid());
            assert!(d.pid_len == 1 || d.pid_len == 2);
            assert!(d.max_width > 0 && d.max_height > 0);
            assert_eq!(describe(d.id), Some(d));
        }
        assert!(describe(SensorId::Hm0360).is_none());
    }

    #[test]
    fn tables_are_non_empty_and_sane() {
        assert!(ov2640::SETTINGS_CIF.len() > 100);
        assert!(ov2640::SETTINGS_JPEG3.len() > 5);
        assert!(ov2640::SETTINGS_YUV422.len() > 3);
        assert!(ov2640::SETTINGS_RGB565.len() > 3);
        // The first step of every OV2640 table selects a bank.
        for t in [
            ov2640::SETTINGS_CIF,
            ov2640::SETTINGS_TO_SVGA,
            ov2640::SETTINGS_TO_UXGA,
        ] {
            assert!(
                matches!(
                    t[0],
                    RegOp::Write {
                        addr: 0xFF,
                        value: 0 | 1
                    }
                ),
                "{:?}",
                t[0]
            );
        }
        assert!(ov5640::DEFAULT_REGS.len() > 100);
        assert!(
            ov5640::FMT_JPEG
                .iter()
                .all(|op| matches!(op, RegOp::Write { .. }))
        );
        // OV5640 addresses are 16-bit and start with the software-reset/system block.
        assert!(matches!(ov5640::DEFAULT_REGS[0], RegOp::Write { addr, .. } if addr >= 0x3000));
    }

    #[test]
    fn modes_and_sizes() {
        let (w, h) = FrameSize::Qvga.dimensions();
        assert_eq!((w, h), (320, 240));
        let m = Mode::jpeg(FrameSize::Uxga).unwrap();
        assert!(m.check(describe(SensorId::Ov2640).unwrap()).is_ok());
        assert_eq!(
            Mode::jpeg(FrameSize::Qxga)
                .unwrap()
                .check(describe(SensorId::Ov2640).unwrap()),
            Err(Error::Unsupported)
        );
        let mut bad = m;
        bad.jpeg_quality = 0;
        assert_eq!(
            bad.check(describe(SensorId::Ov2640).unwrap()),
            Err(Error::InvalidFormat)
        );
    }
}
