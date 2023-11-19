use rp2040_hal::spi::{SpiDevice, ValidSpiPinout};
use usb_device::{class_prelude::UsbBus, UsbError};
use usbd_human_interface_device::interface::{InSize, Interface, OutSize, ReportCount};

use crate::negicon_event::NegiconEvent;

use super::spi::SPIUpstream;
pub(crate) trait Upstream {
    fn send_event(&mut self, event: &NegiconEvent) -> Result<(), UpstreamError>;
}

pub(crate) enum UpstreamError {
    SpiError,
    UsbError(UsbError),
}

impl<B, I, O, R> Upstream for Interface<'_, B, I, O, R>
where
    B: UsbBus,
    I: InSize,
    O: OutSize,
    R: ReportCount,
{
    fn send_event(&mut self, event: &NegiconEvent) -> Result<(), UpstreamError> {
        match self.write_report(&event.serialize()) {
            Ok(_) => Ok(()),
            Err(e) => Err(UpstreamError::UsbError(e)),
        }
    }
}

impl<D, P> Upstream for SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    fn send_event(&mut self, event: &NegiconEvent) -> Result<(), UpstreamError> {
        match self.transmit_event(event) {
            Ok(_) => Ok(()),
            Err(_) => Err(UpstreamError::SpiError),
        }
    }
}
