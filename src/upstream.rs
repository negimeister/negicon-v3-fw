use rp_pico::hal::spi::{SpiDevice, ValidSpiPinout};
use usb_device::class_prelude::UsbBus;
use usbd_human_interface_device::interface::{InSize, Interface, OutSize, ReportCount};

use crate::negicon_event::NegiconEvent;
use crate::spi_upstream::SPIUpstream;
pub(crate) trait Upstream {
    fn send_event(&mut self, event: &NegiconEvent) -> Result<(), &'static str>;
}

impl<B, I, O, R> Upstream for Interface<'_, B, I, O, R>
where
    B: UsbBus,
    I: InSize,
    O: OutSize,
    R: ReportCount,
{
    fn send_event(&mut self, event: &NegiconEvent) -> Result<(), &'static str> {
        match self.write_report(&event.serialize()) {
            Ok(_) => Ok(()),
            Err(_) => Err("USB Error"),
        }
    }
}

impl<D, P> Upstream for SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    fn send_event(&mut self, event: &NegiconEvent) -> Result<(), &'static str> {
        self.transmit_event(event)
    }
}
