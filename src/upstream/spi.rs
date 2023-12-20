use embedded_hal::prelude::_embedded_hal_blocking_spi_Transfer;
use rp2040_hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};



pub(crate) struct SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    spi: Spi<Enabled, D, P, 8>,
}

impl<D, P> SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    pub(crate) fn new(spi: Spi<Enabled, D, P, 8>) -> Self {
        Self { spi }
    }

    pub(crate) fn transmit_event(&mut self, event: &mut [u8; 8]) -> Result<(), &'static str> {
        match self.spi.transfer(event) {
            Ok(_) => Ok(()),
            Err(_) => Err("SPI Upstream Error"),
        }
    }
}
