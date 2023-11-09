use core::cmp::min;

use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::spi::{SpiDevice, ValidSpiPinout};

use crate::{mlx90363::Mlx90363};

pub(crate) struct NegiconEncoder<P, T, U>
where
    P: OutputPin,
    U: ValidSpiPinout<T>,
    T: SpiDevice,
{
    sensor: Mlx90363<P, T, U>,
    min: u16,
    max: u16,
    last: u16,
    deadzone: u16,
}

impl<P, T, U> NegiconEncoder<P, T, U>
where
    P: OutputPin,
    U: ValidSpiPinout<T>,
    T: SpiDevice,
{
    pub(crate) fn new(
        sensor: Mlx90363<P, T, U>,
        min: u16,
        max: u16,
        last: u16,
        deadzone: u16,
    ) -> Self {
        Self {
            sensor,
            min,
            max,
            last,
            deadzone,
        }
    }

    pub(crate) fn calculate_output(&mut self, alpha: u16) -> i16 {
        let diff = min(alpha - self.last, alpha + self.max - self.last);
        if diff < self.deadzone {
            0
        } else {
            diff as i16
        }
    }
}
