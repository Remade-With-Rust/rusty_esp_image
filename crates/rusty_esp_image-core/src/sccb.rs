//! SCCB — the camera control bus. It is I²C without a repeated start: a
//! register read is a write transaction (the address) followed by a separate
//! read transaction. Sensors use 8-bit (OV2640, OV7670) or 16-bit (OV5640,
//! OV3660) register addresses.

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

use crate::sensor::{RegOp, Register, SensorDesc};

/// Register address width of a sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegWidth {
    /// One address byte.
    U8,
    /// Two address bytes, big-endian.
    U16,
}

/// A sensor's control interface over an I²C bus.
#[derive(Debug)]
pub struct Sccb<I> {
    i2c: I,
    addr: u8,
    width: RegWidth,
}

impl<I: I2c> Sccb<I> {
    /// Talk to the sensor at 7-bit address `addr` with `width` register addresses.
    pub fn new(i2c: I, addr: u8, width: RegWidth) -> Self {
        Sccb { i2c, addr, width }
    }

    /// Talk to the sensor `desc` describes.
    pub fn for_sensor(i2c: I, desc: &SensorDesc) -> Self {
        Sccb::new(i2c, desc.sccb_addr, desc.reg_width)
    }

    /// The 7-bit bus address.
    #[must_use]
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Give the bus back.
    pub fn release(self) -> I {
        self.i2c
    }

    fn addr_bytes(&self, reg: u16) -> ([u8; 2], usize) {
        match self.width {
            RegWidth::U8 => ([reg as u8, 0], 1),
            RegWidth::U16 => (reg.to_be_bytes(), 2),
        }
    }

    /// Write one register.
    pub fn write(&mut self, reg: u16, value: u8) -> Result<()> {
        let (a, n) = self.addr_bytes(reg);
        let mut buf = [0u8; 3];
        buf[..n].copy_from_slice(&a[..n]);
        buf[n] = value;
        self.i2c
            .write(self.addr, &buf[..=n])
            .map_err(|_| Error::Hardware)
    }

    /// Write one [`Register`].
    pub fn write_reg(&mut self, reg: Register) -> Result<()> {
        self.write(reg.addr, reg.value)
    }

    /// Read one register (write the address, then read one byte).
    pub fn read(&mut self, reg: u16) -> Result<u8> {
        let (a, n) = self.addr_bytes(reg);
        self.i2c
            .write(self.addr, &a[..n])
            .map_err(|_| Error::Hardware)?;
        let mut v = [0u8; 1];
        self.i2c
            .read(self.addr, &mut v)
            .map_err(|_| Error::Hardware)?;
        Ok(v[0])
    }

    /// Run a register sequence. `delay_ms` is called for each
    /// [`RegOp::DelayMs`] step (the caller owns the clock). Returns the number
    /// of registers written.
    pub fn apply(&mut self, table: &[RegOp], mut delay_ms: impl FnMut(u16)) -> Result<usize> {
        let mut written = 0;
        for op in table {
            match *op {
                RegOp::Write { addr, value } => {
                    self.write(addr, value)?;
                    written += 1;
                }
                RegOp::DelayMs(ms) => delay_ms(ms),
            }
        }
        Ok(written)
    }

    /// Read the product id and compare with `desc`.
    pub fn probe(&mut self, desc: &SensorDesc) -> Result<bool> {
        let pid = match desc.pid_len {
            1 => u16::from(self.read(desc.pid_reg)?),
            2 => {
                let hi = self.read(desc.pid_reg)?;
                let lo = self.read(desc.pid_reg + 1)?;
                u16::from_be_bytes([hi, lo])
            }
            _ => return Err(Error::InvalidFormat),
        };
        Ok(pid == desc.pid)
    }
}

#[cfg(test)]
pub(crate) mod fake {
    //! A register-map I²C device for tests: remembers the last address
    //! written and answers reads from a small map.

    use core::convert::Infallible;

    use embedded_hal::i2c::{ErrorType, I2c, Operation, SevenBitAddress};

    #[derive(Debug, Default)]
    pub struct FakeSensor {
        pub regs: std::collections::BTreeMap<u16, u8>,
        pub last_addr: Option<u16>,
        pub writes: std::vec::Vec<(u16, u8)>,
        pub width16: bool,
    }

    impl ErrorType for FakeSensor {
        type Error = Infallible;
    }

    impl I2c<SevenBitAddress> for FakeSensor {
        fn transaction(
            &mut self,
            _address: SevenBitAddress,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Infallible> {
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        let (reg, rest) = if self.width16 {
                            (u16::from_be_bytes([bytes[0], bytes[1]]), &bytes[2..])
                        } else {
                            (u16::from(bytes[0]), &bytes[1..])
                        };
                        self.last_addr = Some(reg);
                        if let Some(&v) = rest.first() {
                            self.regs.insert(reg, v);
                            self.writes.push((reg, v));
                        }
                    }
                    Operation::Read(buf) => {
                        let reg = self.last_addr.unwrap_or(0);
                        for (i, b) in buf.iter_mut().enumerate() {
                            *b = *self.regs.get(&(reg + i as u16)).unwrap_or(&0);
                        }
                    }
                }
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::FakeSensor;
    use super::*;
    use crate::sensor::{SensorId, describe, ov2640, ov5640};

    #[test]
    fn write_read_apply_probe_8bit() {
        let desc = describe(SensorId::Ov2640).unwrap();
        let mut bus = Sccb::for_sensor(FakeSensor::default(), desc);
        assert_eq!(bus.addr(), 0x30);
        bus.write(0x12, 0x80).unwrap();
        assert_eq!(bus.read(0x12).unwrap(), 0x80);
        let mut delays = 0;
        let n = bus
            .apply(
                &[
                    RegOp::Write {
                        addr: 0x0A,
                        value: 0x26,
                    },
                    RegOp::DelayMs(5),
                    RegOp::Write {
                        addr: 0x0B,
                        value: 0x42,
                    },
                ],
                |_| delays += 1,
            )
            .unwrap();
        assert_eq!((n, delays), (2, 1));
        assert!(bus.probe(desc).unwrap());
        bus.write(0x0A, 0x00).unwrap();
        assert!(!bus.probe(desc).unwrap());
        // the real init table applies without error and writes every step
        let n = bus.apply(ov2640::SETTINGS_CIF, |_| {}).unwrap();
        assert_eq!(n, ov2640::SETTINGS_CIF.len());
        let inner = bus.release();
        assert!(inner.writes.len() > 100);
    }

    #[test]
    fn sixteen_bit_addresses() {
        let desc = describe(SensorId::Ov5640).unwrap();
        let mut bus = Sccb::for_sensor(
            FakeSensor {
                width16: true,
                ..FakeSensor::default()
            },
            desc,
        );
        bus.write(0x300A, 0x56).unwrap();
        bus.write(0x300B, 0x40).unwrap();
        assert_eq!(bus.read(0x300A).unwrap(), 0x56);
        assert!(bus.probe(desc).unwrap());
        let mut delays = std::vec::Vec::new();
        let n = bus
            .apply(ov5640::DEFAULT_REGS, |ms| delays.push(ms))
            .unwrap();
        let writes = ov5640::DEFAULT_REGS
            .iter()
            .filter(|op| matches!(op, RegOp::Write { .. }))
            .count();
        assert_eq!(n, writes);
        assert_eq!(
            delays.len(),
            ov5640::DEFAULT_REGS.len() - writes,
            "every delay step reached the caller's clock"
        );
    }
}
