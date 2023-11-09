use embedded_hal::prelude::_embedded_hal_blocking_spi_Transfer;
use rp_pico::hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};

use crate::negicon_event::NegiconEvent;

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

    pub(crate) fn transmit_event(&mut self, event: &NegiconEvent) -> Result<(), &'static str> {
        match self.spi.transfer(&mut event.serialize()) {
            Ok(_) => Ok(()),
            Err(_) => Err("SPI Upstream Error"),
        }
    }
}
