//! The OV3660 as esp32-camera's `ov3660.c` drives it: the same register
//! sequence, computed the same way, over the tables in
//! [`crate::sensor::ov3660`].
//!
//! `reset` is the C `reset()`: a software reset, the default table, the
//! auto-exposure target. `configure` is what `esp_camera_init` does after
//! it — `set_framesize` (the window, binning and scaling, the timing
//! options, the PLL for the format), `set_pixformat`, `set_quality` for
//! JPEG — in that order, with the format known to `set_framesize` as the C
//! has it. Sixteen-bit registers throughout.

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::frame::PixelFormat;

use crate::sccb::{RegWidth, Sccb};
use crate::sensor::ov3660::{
    DEFAULT_REGS, FMT_GRAYSCALE, FMT_JPEG, FMT_RGB565, FMT_YUV422, RATIO_TABLE,
};
use crate::sensor::{FrameSize, Mode};

/// 7-bit SCCB address.
pub const SCCB_ADDR: u8 = 0x3C;
/// The product id at `0x300A`/`0x300B`.
pub const PID: u16 = 0x3660;

// The registers this driver writes by name, as `ov3660_regs.h` names them.
const SYSTEM_CTROL0: u16 = 0x3008;
const PID_H: u16 = 0x300A;
const SC_PLLS_CTRL0: u16 = 0x303A;
const SC_PLLS_CTRL1: u16 = 0x303B;
const SC_PLLS_CTRL2: u16 = 0x303C;
const SC_PLLS_CTRL3: u16 = 0x303D;
const X_ADDR_ST_H: u16 = 0x3800;
const X_ADDR_END_H: u16 = 0x3804;
const X_OUTPUT_SIZE_H: u16 = 0x3808;
const X_TOTAL_SIZE_H: u16 = 0x380C;
const X_OFFSET_H: u16 = 0x3810;
const X_INCREMENT: u16 = 0x3814;
const Y_INCREMENT: u16 = 0x3815;
const TIMING_TC_REG20: u16 = 0x3820;
const TIMING_TC_REG21: u16 = 0x3821;
const PCLK_RATIO: u16 = 0x3824;
const REG_4514: u16 = 0x4514;
const REG_4520: u16 = 0x4520;
const COMPRESSION_CTRL07: u16 = 0x4407;
const VFIFO_CTRL0C: u16 = 0x460C;
const ISP_CONTROL_01: u16 = 0x5001;
const PRE_ISP_TEST_SETTING_1: u16 = 0x503D;
/// The auto-exposure target registers `set_ae_level` writes, in its order.
const AE_LEVEL_REGS: [u16; 6] = [0x3A0F, 0x3A10, 0x3A1B, 0x3A1E, 0x3A11, 0x3A1F];

/// The sensor's test pattern, register `0x503D`: what the X5 oracle
/// compares two drivers on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestPattern {
    /// The picture.
    Off,
    /// Eight standard colour bars, still.
    ColourBars,
    /// The bars rolling — what esp32-camera's `set_colorbar(1)` selects.
    RollingColourBars,
}

impl TestPattern {
    const fn bits(self) -> u8 {
        match self {
            TestPattern::Off => 0x00,
            TestPattern::ColourBars => 0x80,
            TestPattern::RollingColourBars => 0xC0,
        }
    }
}

/// An OV3660 on an I²C bus, with the state the C driver keeps beside it.
pub struct Ov3660<I> {
    bus: Sccb<I>,
    xclk_hz: u32,
    format: PixelFormat,
    binning: bool,
    vflip: bool,
    hmirror: bool,
}

impl<I> core::fmt::Debug for Ov3660<I> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Ov3660")
            .field("xclk_hz", &self.xclk_hz)
            .field("format", &self.format)
            .field("binning", &self.binning)
            .finish_non_exhaustive()
    }
}

impl<I: I2c> Ov3660<I> {
    /// A sensor on `i2c`, fed `xclk_hz` on its master clock (the PLL
    /// choice for JPEG depends on it).
    pub fn new(i2c: I, xclk_hz: u32) -> Self {
        Ov3660 {
            bus: Sccb::new(i2c, SCCB_ADDR, RegWidth::U16),
            xclk_hz,
            format: PixelFormat::Jpeg,
            binning: false,
            vflip: false,
            hmirror: false,
        }
    }

    /// Give the bus back.
    pub fn release(self) -> I {
        self.bus.release()
    }

    /// The bus, for a register the driver does not name.
    pub fn bus(&mut self) -> &mut Sccb<I> {
        &mut self.bus
    }

    /// Whether the part on the bus answers with the OV3660's product id.
    pub fn probe(&mut self) -> Result<bool> {
        let h = self.bus.read(PID_H)?;
        let l = self.bus.read(PID_H + 1)?;
        Ok(u16::from_be_bytes([h, l]) == PID)
    }

    /// The C `reset()`: software reset, the default table, the exposure
    /// target. `delay_ms` is the caller's clock.
    pub fn reset(&mut self, delay_ms: &mut impl FnMut(u16)) -> Result<()> {
        self.bus.write(SYSTEM_CTROL0, 0x82)?;
        delay_ms(100);
        self.bus.apply(DEFAULT_REGS, &mut *delay_ms)?;
        self.set_ae_level(0)?;
        delay_ms(100);
        Ok(())
    }

    /// What `esp_camera_init` does after the reset, for `mode`: the frame
    /// size (with the format already known, as the C has it), the pixel
    /// format, the JPEG quality.
    pub fn configure(&mut self, mode: &Mode) -> Result<()> {
        let size = FrameSize::from_dimensions(mode.geometry.width, mode.geometry.height)
            .ok_or(Error::Unsupported)?;
        if size.ordinal() > FrameSize::Qxga.ordinal() {
            return Err(Error::Unsupported);
        }
        self.format = mode.geometry.format;
        self.vflip = mode.flip;
        self.hmirror = mode.mirror;
        self.set_framesize(size)?;
        self.set_pixformat(self.format)?;
        if self.format == PixelFormat::Jpeg {
            self.set_quality(mode.jpeg_quality)?;
        }
        Ok(())
    }

    /// The C `set_framesize()`: the sensor window for the size's aspect
    /// ratio, binning when the size is at most half the window, the ISP
    /// scaler when it is neither the window nor its half, the timing
    /// options, and the PLL for the format and size.
    pub fn set_framesize(&mut self, size: FrameSize) -> Result<()> {
        let (w, h) = size.dimensions();
        let (w, h) = (w as u16, h as u16);
        let r = RATIO_TABLE[size.aspect_ratio() as usize];
        self.binning = w <= r.max_width / 2 && h <= r.max_height / 2;
        let scale = !((w == r.max_width && h == r.max_height)
            || (w == r.max_width / 2 && h == r.max_height / 2));

        self.write_addr(X_ADDR_ST_H, r.start_x, r.start_y)?;
        self.write_addr(X_ADDR_END_H, r.end_x, r.end_y)?;
        self.write_addr(X_OUTPUT_SIZE_H, w, h)?;
        if self.binning {
            self.write_addr(X_TOTAL_SIZE_H, r.total_x, r.total_y / 2 + 1)?;
            self.write_addr(X_OFFSET_H, 8, 2)?;
        } else {
            self.write_addr(X_TOTAL_SIZE_H, r.total_x, r.total_y)?;
            self.write_addr(X_OFFSET_H, 16, 6)?;
        }
        self.write_bits(ISP_CONTROL_01, 0x20, scale)?;
        self.set_image_options()?;

        if self.format == PixelFormat::Jpeg {
            if size == FrameSize::Qxga || self.xclk_hz == 16_000_000 {
                // 40 MHz SYSCLK and 10 MHz PCLK
                self.set_pll(false, 24, 1, 3, false, 0, true, 8)
            } else {
                // 50 MHz SYSCLK and 10 MHz PCLK
                self.set_pll(false, 30, 1, 3, false, 0, true, 10)
            }
        } else if size.ordinal() > FrameSize::Hvga.ordinal() {
            // 8 MHz SYSCLK and 8 MHz PCLK
            self.set_pll(false, 4, 1, 0, false, 2, true, 2)
        } else if size.ordinal() >= FrameSize::Qvga.ordinal() {
            // 16 MHz SYSCLK and 8 MHz PCLK
            self.set_pll(false, 8, 1, 0, false, 2, true, 4)
        } else {
            // 32 MHz SYSCLK and 8 MHz PCLK
            self.set_pll(false, 8, 1, 0, false, 0, true, 8)
        }
    }

    /// The C `set_image_options()`: compression, binning, flip and mirror
    /// into the two timing registers and `0x4514`, then the subsample
    /// increments.
    fn set_image_options(&mut self) -> Result<()> {
        let mut reg20 = 0u8;
        let mut reg21 = 0u8;
        let mut test = 0u8;
        if self.format == PixelFormat::Jpeg {
            reg21 |= 0x20;
        }
        if self.binning {
            reg20 |= 0x01;
            reg21 |= 0x01;
            test |= 4;
        } else {
            reg20 |= 0x40;
        }
        if self.vflip {
            reg20 |= 0x06;
            test |= 1;
        }
        if self.hmirror {
            reg21 |= 0x06;
            test |= 2;
        }
        let reg4514 = match test {
            0 | 1 => 0x88,
            2 | 3 => 0xBB,
            4 | 7 => 0xAA,
            _ => 0xBB,
        };
        self.bus.write(TIMING_TC_REG20, reg20)?;
        self.bus.write(TIMING_TC_REG21, reg21)?;
        self.bus.write(REG_4514, reg4514)?;
        if self.binning {
            self.bus.write(REG_4520, 0x0B)?;
            self.bus.write(X_INCREMENT, 0x31)?;
            self.bus.write(Y_INCREMENT, 0x31)
        } else {
            self.bus.write(REG_4520, 0xB0)?;
            self.bus.write(X_INCREMENT, 0x11)?;
            self.bus.write(Y_INCREMENT, 0x11)
        }
    }

    /// The C `set_pll()`: the six clock registers.
    #[allow(clippy::too_many_arguments)]
    fn set_pll(
        &mut self,
        bypass: bool,
        multiplier: u8,
        sys_div: u8,
        pre_div: u8,
        root_2x: bool,
        seld5: u8,
        pclk_manual: bool,
        pclk_div: u8,
    ) -> Result<()> {
        if multiplier > 31 || sys_div > 15 || pre_div > 3 || pclk_div > 31 || seld5 > 3 {
            return Err(Error::InvalidFormat);
        }
        self.bus
            .write(SC_PLLS_CTRL0, if bypass { 0x80 } else { 0x00 })?;
        self.bus.write(SC_PLLS_CTRL1, multiplier & 0x1F)?;
        self.bus.write(SC_PLLS_CTRL2, 0x10 | (sys_div & 0x0F))?;
        self.bus.write(
            SC_PLLS_CTRL3,
            ((pre_div & 0x3) << 4) | seld5 | if root_2x { 0x40 } else { 0x00 },
        )?;
        self.bus.write(PCLK_RATIO, pclk_div & 0x1F)?;
        self.bus
            .write(VFIFO_CTRL0C, if pclk_manual { 0x22 } else { 0x20 })
    }

    /// The C `set_pixformat()`: the format's table.
    pub fn set_pixformat(&mut self, format: PixelFormat) -> Result<()> {
        let table = match format {
            PixelFormat::Yuyv422 => FMT_YUV422,
            PixelFormat::Gray8 => FMT_GRAYSCALE,
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => FMT_RGB565,
            PixelFormat::Jpeg => FMT_JPEG,
            _ => return Err(Error::Unsupported),
        };
        self.bus.apply(table, |_| {})?;
        self.format = format;
        Ok(())
    }

    /// The C `set_quality()`: the JPEG quantisation scale, 0–63.
    pub fn set_quality(&mut self, qs: u8) -> Result<()> {
        self.bus.write(COMPRESSION_CTRL07, qs & 0x3F)
    }

    /// The test pattern. esp32-camera's `set_colorbar(1)` is
    /// [`TestPattern::RollingColourBars`]; the still bars are the oracle,
    /// and a C driver reaches them through its `set_reg` with the same mask.
    pub fn set_test_pattern(&mut self, pattern: TestPattern) -> Result<()> {
        self.set_reg(PRE_ISP_TEST_SETTING_1, 0xC0, pattern.bits())
    }

    /// The C `set_reg()`: `reg = (reg & !mask) | (value & mask)`.
    pub fn set_reg(&mut self, reg: u16, mask: u8, value: u8) -> Result<()> {
        let current = self.bus.read(reg)?;
        self.bus.write(reg, (current & !mask) | (value & mask))
    }

    /// The C `set_ae_level()`: the exposure target and its windows, for
    /// `level` in −5..=5.
    pub fn set_ae_level(&mut self, level: i8) -> Result<()> {
        if !(-5..=5).contains(&level) {
            return Err(Error::InvalidFormat);
        }
        let target = (i32::from(level) + 5) * 10 + 5;
        let low = target * 23 / 25;
        let high = target * 27 / 25;
        let fast_low = low >> 1;
        let fast_high = (high << 1).min(255);
        let values = [high, low, high, low, fast_high, fast_low];
        for (reg, value) in AE_LEVEL_REGS.iter().zip(values) {
            self.bus.write(*reg, value as u8)?;
        }
        Ok(())
    }

    fn write_bits(&mut self, reg: u16, mask: u8, on: bool) -> Result<()> {
        self.set_reg(reg, mask, if on { mask } else { 0 })
    }

    fn write16(&mut self, reg: u16, value: u16) -> Result<()> {
        self.bus.write(reg, (value >> 8) as u8)?;
        self.bus.write(reg + 1, value as u8)
    }

    /// The C `write_addr_reg()`: an x/y pair of 16-bit registers.
    fn write_addr(&mut self, reg: u16, x: u16, y: u16) -> Result<()> {
        self.write16(reg, x)?;
        self.write16(reg + 2, y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sccb::fake::FakeSensor;
    use crate::sensor::RegOp;
    use std::vec::Vec;

    fn sensor() -> Ov3660<FakeSensor> {
        let mut fake = FakeSensor {
            width16: true,
            ..FakeSensor::default()
        };
        fake.regs.insert(0x300A, 0x36);
        fake.regs.insert(0x300B, 0x60);
        Ov3660::new(fake, 20_000_000)
    }

    fn table_writes(table: &[RegOp]) -> Vec<(u16, u8)> {
        table
            .iter()
            .filter_map(|op| match *op {
                RegOp::Write { addr, value } => Some((addr, value)),
                RegOp::DelayMs(_) => None,
            })
            .collect()
    }

    #[test]
    fn probes_by_product_id() {
        let mut s = sensor();
        assert!(s.probe().unwrap());
        s.bus().write(0x300B, 0x40).unwrap();
        assert!(!s.probe().unwrap());
    }

    /// The sequence `esp_camera_init` leaves on the bus for QVGA JPEG at
    /// quality 12 with a 20 MHz clock, derived by hand from `ov3660.c`.
    #[test]
    fn qvga_jpeg_is_the_c_drivers_sequence() {
        let mut s = sensor();
        let mut delays = Vec::new();
        s.reset(&mut |ms| delays.push(ms)).unwrap();
        s.configure(&Mode::jpeg(FrameSize::Qvga).unwrap()).unwrap();
        let writes = s.release().writes;

        // the reset: software reset, the default table, the exposure target
        assert_eq!(writes[0], (0x3008, 0x82));
        let defaults = table_writes(DEFAULT_REGS);
        assert_eq!(&writes[1..1 + defaults.len()], &defaults[..]);
        // 100 ms, the table's own 10 ms, 100 ms
        assert_eq!(delays, [100, 10, 100]);
        let mut i = 1 + defaults.len();
        // level 0: target 55 → high 59, low 50, fast 118 / 25
        let ae = [
            (0x3A0F, 59),
            (0x3A10, 50),
            (0x3A1B, 59),
            (0x3A1E, 50),
            (0x3A11, 118),
            (0x3A1F, 25),
        ];
        assert_eq!(&writes[i..i + ae.len()], &ae[..]);
        i += ae.len();

        // set_framesize(QVGA): the 4:3 window, 320×240, binning (≤ half
        // the window) with the halved total and the 8/2 offset, the scaler
        // (neither the window nor its half), the timing options, the JPEG
        // PLL at 20 MHz
        let window = [
            (0x3800, 0x00),
            (0x3801, 0x00),
            (0x3802, 0x00),
            (0x3803, 0x00),
            (0x3804, 0x08),
            (0x3805, 0x1F),
            (0x3806, 0x06),
            (0x3807, 0x0B),
            (0x3808, 0x01),
            (0x3809, 0x40),
            (0x380A, 0x00),
            (0x380B, 0xF0),
            (0x380C, 0x08),
            (0x380D, 0xFC),
            (0x380E, 0x03),
            (0x380F, 0x0F),
            (0x3810, 0x00),
            (0x3811, 0x08),
            (0x3812, 0x00),
            (0x3813, 0x02),
        ];
        assert_eq!(&writes[i..i + window.len()], &window[..]);
        i += window.len();
        assert_eq!(writes[i].0, 0x5001);
        assert_eq!(writes[i].1 & 0x20, 0x20, "the scaler is enabled");
        i += 1;
        let options = [
            (0x3820, 0x01),
            (0x3821, 0x21),
            (0x4514, 0xAA),
            (0x4520, 0x0B),
            (0x3814, 0x31),
            (0x3815, 0x31),
        ];
        assert_eq!(&writes[i..i + options.len()], &options[..]);
        i += options.len();
        let pll = [
            (0x303A, 0x00),
            (0x303B, 30),
            (0x303C, 0x11),
            (0x303D, 0x30),
            (0x3824, 10),
            (0x460C, 0x22),
        ];
        assert_eq!(&writes[i..i + pll.len()], &pll[..]);
        i += pll.len();

        // set_pixformat(JPEG), then the quality
        let fmt = table_writes(FMT_JPEG);
        assert_eq!(&writes[i..i + fmt.len()], &fmt[..]);
        i += fmt.len();
        assert_eq!(writes[i], (0x4407, 12));
        assert_eq!(writes.len(), i + 1, "nothing else was written");
    }

    #[test]
    fn vga_rgb565_takes_the_raw_pll_and_no_compression_bit() {
        let mut s = sensor();
        s.reset(&mut |_| {}).unwrap();
        s.configure(&Mode::raw(FrameSize::Vga, PixelFormat::Rgb565).unwrap())
            .unwrap();
        let writes = s.release().writes;
        let tail: Vec<(u16, u8)> = writes.iter().rev().take(20).rev().copied().collect();
        // binning (640 ≤ 1024, 480 ≤ 768), no compression: 0x3821 = 0x01
        assert!(tail.contains(&(0x3821, 0x01)));
        // larger than HVGA, not JPEG: multiplier 4, seld5 2, pclk_div 2
        assert!(tail.contains(&(0x303B, 4)));
        assert!(tail.contains(&(0x303D, 0x02)));
        assert!(tail.contains(&(0x3824, 2)));
        // the RGB565 format table, and no quality write
        assert_eq!(tail.last(), Some(&(0x4300, 0x61)));
    }

    #[test]
    fn the_test_pattern_masks_two_bits() {
        let mut s = sensor();
        s.bus().write(0x503D, 0x3F).unwrap();
        s.set_test_pattern(TestPattern::ColourBars).unwrap();
        s.set_test_pattern(TestPattern::RollingColourBars).unwrap();
        s.set_test_pattern(TestPattern::Off).unwrap();
        let writes = s.release().writes;
        assert_eq!(
            &writes[1..],
            &[(0x503D, 0xBF), (0x503D, 0xFF), (0x503D, 0x3F)]
        );
    }

    #[test]
    fn refuses_what_the_part_cannot_do() {
        let mut s = sensor();
        assert_eq!(
            s.configure(&Mode::jpeg(FrameSize::Qsxga).unwrap()),
            Err(Error::Unsupported)
        );
        assert_eq!(s.set_ae_level(6), Err(Error::InvalidFormat));
    }
}
