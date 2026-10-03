//! The OV2640 as esp32-camera's `ov2640.c` drives it: eight-bit registers
//! in two banks selected through `0xFF`, the tables in
//! [`crate::sensor::ov2640`], and the window and clock registers computed
//! as the C computes them.
//!
//! `reset` is the C `reset()`; `configure` is what `esp_camera_init` does
//! after it for this part — `set_framesize` (which re-applies the format
//! table, as the C does), `set_pixformat`, the four defaults the init
//! function sets only for the OV2640 (gain ceiling 2×, black-pixel
//! correction off, white-pixel correction and lens correction on), and
//! `set_quality` for JPEG. The bank register is cached as the C caches it,
//! so the writes on the bus are the C's writes.
//!
//! Built alongside the OV3660 and not yet run: the board on the bench
//! carries the other part.

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::PixelFormat;

use crate::sccb::{RegWidth, Sccb};
use crate::sensor::ov2640::{
    SETTINGS_CIF, SETTINGS_JPEG3, SETTINGS_RGB565, SETTINGS_TO_CIF, SETTINGS_TO_SVGA,
    SETTINGS_TO_UXGA, SETTINGS_YUV422,
};
use crate::sensor::{FrameSize, Mode, RegOp};

/// 7-bit SCCB address.
pub const SCCB_ADDR: u8 = 0x30;
/// The product id high byte at `0x0A` in the sensor bank.
pub const PID: u8 = 0x26;

const BANK_SEL: u16 = 0xFF;
const BANK_DSP: u8 = 0;
const BANK_SENSOR: u8 = 1;

// DSP bank
const R_BYPASS: u16 = 0x05;
const QS: u16 = 0x44;
const HSIZE: u16 = 0x51;
const VSIZE: u16 = 0x52;
const XOFFL: u16 = 0x53;
const YOFFL: u16 = 0x54;
const VHYX: u16 = 0x55;
const TEST: u16 = 0x57;
const ZMOW: u16 = 0x5A;
const ZMOH: u16 = 0x5B;
const ZMHH: u16 = 0x5C;
const CTRL3: u16 = 0x87;
const CTRL1: u16 = 0xC3;
const R_DVP_SP: u16 = 0xD3;
// sensor bank
const PIDH: u16 = 0x0A;
const CLKRC: u16 = 0x11;
const COM7: u16 = 0x12;
const COM9: u16 = 0x14;
const COM7_SRST: u8 = 0x80;
const COM7_COLOR_BAR: u8 = 0x02;

/// The sensor window for each aspect ratio, `ratio_table` in
/// `ov2640_settings.h`: offset x/y, then the window's width and height.
const RATIO_TABLE: [(u16, u16, u16, u16); 9] = [
    (0, 0, 1600, 1200),   // 4x3
    (8, 72, 1584, 1056),  // 3x2
    (0, 100, 1600, 1000), // 16x10
    (0, 120, 1600, 960),  // 5x3
    (0, 150, 1600, 900),  // 16x9
    (2, 258, 1596, 684),  // 21x9
    (50, 0, 1500, 1200),  // 5x4
    (200, 0, 1200, 1200), // 1x1
    (462, 0, 676, 1200),  // 9x16
];

/// Which of the three sensor modes a size runs in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SensorMode {
    Cif,
    Svga,
    Uxga,
}

/// An OV2640 on an I²C bus.
pub struct Ov2640<I> {
    bus: Sccb<I>,
    bank: Option<u8>,
    format: PixelFormat,
}

impl<I> core::fmt::Debug for Ov2640<I> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Ov2640")
            .field("format", &self.format)
            .finish_non_exhaustive()
    }
}

impl<I: I2c> Ov2640<I> {
    /// A sensor on `i2c`.
    pub fn new(i2c: I) -> Self {
        Ov2640 {
            bus: Sccb::new(i2c, SCCB_ADDR, RegWidth::U8),
            bank: None,
            format: PixelFormat::Jpeg,
        }
    }

    /// Give the bus back.
    pub fn release(self) -> I {
        self.bus.release()
    }

    /// Whether the part on the bus answers with the OV2640's product id.
    pub fn probe(&mut self) -> Result<bool> {
        self.set_bank(BANK_SENSOR)?;
        Ok(self.bus.read(PIDH)? == PID)
    }

    /// The C `reset()`: a software reset in the sensor bank, then the CIF
    /// table.
    pub fn reset(&mut self, delay_ms: &mut impl FnMut(u16)) -> Result<()> {
        self.write_reg(BANK_SENSOR, COM7, COM7_SRST)?;
        delay_ms(10);
        self.write_regs(SETTINGS_CIF, delay_ms)
    }

    /// What `esp_camera_init` does after the reset for this part.
    pub fn configure(&mut self, mode: &Mode, delay_ms: &mut impl FnMut(u16)) -> Result<()> {
        let size = FrameSize::from_dimensions(mode.geometry.width, mode.geometry.height)
            .ok_or(Error::Unsupported)?;
        if size.ordinal() > FrameSize::Uxga.ordinal() {
            return Err(Error::Unsupported);
        }
        self.format = mode.geometry.format;
        self.set_framesize(size, delay_ms)?;
        self.set_pixformat(self.format, delay_ms)?;
        // the OV2640-only defaults, in the init function's order
        self.set_reg_bits(BANK_SENSOR, COM9, 5, 7, 0)?; // gain ceiling 2x
        self.set_reg_bits(BANK_DSP, CTRL3, 7, 1, 0)?; // black-pixel correction off
        self.set_reg_bits(BANK_DSP, CTRL3, 6, 1, 1)?; // white-pixel correction on
        self.set_reg_bits(BANK_DSP, CTRL1, 1, 1, 1)?; // lens correction on
        if self.format == PixelFormat::Jpeg {
            self.set_quality(mode.jpeg_quality)?;
        }
        Ok(())
    }

    /// The C `set_pixformat()`: the format's table, then 10 ms.
    pub fn set_pixformat(
        &mut self,
        format: PixelFormat,
        delay_ms: &mut impl FnMut(u16),
    ) -> Result<()> {
        let table = match format {
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => SETTINGS_RGB565,
            PixelFormat::Yuyv422 | PixelFormat::Gray8 => SETTINGS_YUV422,
            PixelFormat::Jpeg => SETTINGS_JPEG3,
            _ => return Err(Error::Unsupported),
        };
        self.format = format;
        self.write_regs(table, delay_ms)?;
        delay_ms(10);
        Ok(())
    }

    /// The C `set_framesize()`: the window for the size's aspect ratio in
    /// the sensor mode the size needs, through `set_window`.
    pub fn set_framesize(&mut self, size: FrameSize, delay_ms: &mut impl FnMut(u16)) -> Result<()> {
        let (w, h) = size.dimensions();
        let (mut offset_x, mut offset_y, mut max_x, mut max_y) =
            RATIO_TABLE[size.aspect_ratio() as usize];
        let mode = if size.ordinal() <= FrameSize::Cif.ordinal() {
            max_x /= 4;
            max_y /= 4;
            offset_x /= 4;
            offset_y /= 4;
            if max_y > 296 {
                max_y = 296;
            }
            SensorMode::Cif
        } else if size.ordinal() <= FrameSize::Svga.ordinal() {
            max_x /= 2;
            max_y /= 2;
            offset_x /= 2;
            offset_y /= 2;
            SensorMode::Svga
        } else {
            SensorMode::Uxga
        };
        self.set_window(
            mode, offset_x, offset_y, max_x, max_y, w as u16, h as u16, delay_ms,
        )
    }

    /// The C `set_window()`: the DSP bypassed, the mode's table, the
    /// window registers, the sensor and pixel clocks, the DSP back, then
    /// the format table again ("required when changing resolution").
    #[allow(clippy::too_many_arguments)]
    fn set_window(
        &mut self,
        mode: SensorMode,
        offset_x: u16,
        offset_y: u16,
        max_x: u16,
        max_y: u16,
        w: u16,
        h: u16,
        delay_ms: &mut impl FnMut(u16),
    ) -> Result<()> {
        let (max_x, max_y, w, h) = (max_x / 4, max_y / 4, w / 4, h / 4);
        let window: [(u16, u8); 9] = [
            (HSIZE, (max_x & 0xFF) as u8),
            (VSIZE, (max_y & 0xFF) as u8),
            (XOFFL, (offset_x & 0xFF) as u8),
            (YOFFL, (offset_y & 0xFF) as u8),
            (
                VHYX,
                (((max_y >> 1) & 0x80)
                    | ((offset_y >> 4) & 0x70)
                    | ((max_x >> 5) & 0x08)
                    | ((offset_x >> 8) & 0x07)) as u8,
            ),
            (TEST, ((max_x >> 2) & 0x80) as u8),
            (ZMOW, (w & 0xFF) as u8),
            (ZMOH, (h & 0xFF) as u8),
            (ZMHH, (((h >> 6) & 0x04) | ((w >> 8) & 0x03)) as u8),
        ];

        // the clock registers: `clk` is CLKRC (divider, 2× in bit 7),
        // `pclk` is R_DVP_SP (divider, auto in bit 7)
        let (clk_2x, clk_div, pclk_auto, pclk_div) = if self.format == PixelFormat::Jpeg {
            (
                false,
                0u8,
                false,
                if mode == SensorMode::Uxga { 12u8 } else { 8 },
            )
        } else {
            let clk_div = if mode == SensorMode::Cif { 3 } else { 7 };
            let pclk_div = if mode == SensorMode::Uxga { 12 } else { 8 };
            // the 2× is what every target but the original ESP32 sets
            (true, clk_div, true, pclk_div)
        };
        let clk = (clk_div & 0x3F) | if clk_2x { 0x80 } else { 0 };
        let pclk = (pclk_div & 0x7F) | if pclk_auto { 0x80 } else { 0 };

        let regs = match mode {
            SensorMode::Cif => SETTINGS_TO_CIF,
            SensorMode::Svga => SETTINGS_TO_SVGA,
            SensorMode::Uxga => SETTINGS_TO_UXGA,
        };
        self.write_reg(BANK_DSP, R_BYPASS, 0x01)?;
        self.write_regs(regs, delay_ms)?;
        self.set_bank(BANK_DSP)?;
        for (reg, value) in window {
            self.bus.write(reg, value)?;
        }
        self.write_reg(BANK_SENSOR, CLKRC, clk)?;
        self.write_reg(BANK_DSP, R_DVP_SP, pclk)?;
        self.write_reg(BANK_DSP, R_BYPASS, 0x00)?;
        delay_ms(10);
        self.set_pixformat(self.format, delay_ms)
    }

    /// The C `set_quality()`: the DSP's quantisation scale, 0–63.
    pub fn set_quality(&mut self, quality: u8) -> Result<()> {
        self.write_reg(BANK_DSP, QS, quality.min(63))
    }

    /// The C `set_colorbar()`: bit 1 of COM7.
    pub fn set_colorbar(&mut self, on: bool) -> Result<()> {
        self.set_reg_bits(
            BANK_SENSOR,
            COM7,
            0,
            COM7_COLOR_BAR,
            if on { COM7_COLOR_BAR } else { 0 },
        )
    }

    /// Select `bank` if it is not the one selected — the C driver's cache.
    fn set_bank(&mut self, bank: u8) -> Result<()> {
        if self.bank != Some(bank) {
            self.bus.write(BANK_SEL, bank)?;
            self.bank = Some(bank);
        }
        Ok(())
    }

    fn write_reg(&mut self, bank: u8, reg: u16, value: u8) -> Result<()> {
        self.set_bank(bank)?;
        self.bus.write(reg, value)
    }

    /// The C `set_reg_bits()`.
    fn set_reg_bits(&mut self, bank: u8, reg: u16, offset: u8, mask: u8, value: u8) -> Result<()> {
        self.set_bank(bank)?;
        let current = self.bus.read(reg)?;
        let new = (current & !(mask << offset)) | ((value & mask) << offset);
        self.bus.write(reg, new)
    }

    /// The C `write_regs()`: a table, with bank selects through the cache.
    fn write_regs(&mut self, table: &[RegOp], delay_ms: &mut impl FnMut(u16)) -> Result<()> {
        for op in table {
            match *op {
                RegOp::Write { addr, value } if addr == BANK_SEL => self.set_bank(value)?,
                RegOp::Write { addr, value } => self.bus.write(addr, value)?,
                RegOp::DelayMs(ms) => delay_ms(ms),
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sccb::fake::FakeSensor;
    use std::vec::Vec;

    fn sensor() -> Ov2640<FakeSensor> {
        let mut fake = FakeSensor::default();
        fake.regs.insert(0x0A, 0x26);
        Ov2640::new(fake)
    }

    #[test]
    fn probes_in_the_sensor_bank() {
        let mut s = sensor();
        assert!(s.probe().unwrap());
        let writes = s.release().writes;
        assert_eq!(writes, [(0xFF, 0x01)]);
    }

    #[test]
    fn reset_then_qvga_jpeg_writes_the_c_drivers_window_and_clocks() {
        let mut s = sensor();
        let mut delays = Vec::new();
        s.reset(&mut |ms| delays.push(ms)).unwrap();
        s.configure(&Mode::jpeg(FrameSize::Qvga).unwrap(), &mut |ms| {
            delays.push(ms)
        })
        .unwrap();
        let writes = s.release().writes;
        // the reset: the sensor bank, COM7 = 0x80, then the CIF table
        assert_eq!(&writes[..2], &[(0xFF, 0x01), (0x12, 0x80)]);
        // QVGA is CIF mode: the 4:3 window quartered (1600×1200 → 400×296),
        // then /4 for the registers: 100×74, output 80×60
        let window = [
            (0x51, 100),
            (0x52, 74),
            (0x53, 0),
            (0x54, 0),
            (0x55, 0),
            (0x57, 0),
            (0x5A, 80),
            (0x5B, 60),
            (0x5C, 0),
        ];
        let at = writes
            .windows(window.len())
            .position(|w| w == window)
            .expect("the window registers, in order");
        // then CLKRC (JPEG: divider 0, no 2×) in the sensor bank and
        // R_DVP_SP (divider 8, manual) in the DSP bank, the DSP back on
        assert_eq!(
            &writes[at + window.len()..at + window.len() + 5],
            &[
                (0xFF, 0x01),
                (0x11, 0x00),
                (0xFF, 0x00),
                (0xD3, 0x08),
                (0x05, 0x00)
            ]
        );
        // the quality last
        assert_eq!(writes.last(), Some(&(0x44, 12)));
        // 10 ms after the reset, 10 after the window, 10 after each format table
        assert!(delays.iter().all(|&d| d == 10));
        assert!(delays.len() >= 4);
    }

    #[test]
    fn the_colour_bar_is_one_bit_of_com7() {
        let mut s = sensor();
        s.set_colorbar(true).unwrap();
        s.set_colorbar(false).unwrap();
        let writes = s.release().writes;
        assert_eq!(writes, [(0xFF, 0x01), (0x12, 0x02), (0x12, 0x00)]);
    }
}
