use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::spi::{SpiDevice, ValidSpiPinout};

use crate::mlx90363::Mlx90363;

trait Downstream {
    fn poll(&mut self) -> Result<(), ()>;
}

impl<P, T, U> Downstream for Mlx90363<P, T, U>
where
    P: OutputPin,
    T: SpiDevice,
    U: ValidSpiPinout<T>,
{
    fn poll(&mut self) -> Result<(), ()> {
        match self.get_alpha() {
            Ok(_) => todo!(),
            Err(_) => todo!(),
        }
    }
}
