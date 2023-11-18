
use rp_pico::hal::spi::{SpiDevice, ValidSpiPinout};
use usb_device::class_prelude::UsbBus;
use usbd_human_interface_device::interface::{InSize, Interface, OutSize, ReportCount};

use crate::negicon_event::NegiconEvent;

use super::spi::SPIUpstream;
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
            Err(e) => match e {
                usb_device::UsbError::WouldBlock => Err("USB WouldBlock"),
                usb_device::UsbError::ParseError => Err("USB ParseError"),
                usb_device::UsbError::BufferOverflow => Err("USB BufferOverflow"),
                usb_device::UsbError::EndpointOverflow => Err("USB EndpointOverflow"),
                usb_device::UsbError::InvalidEndpoint => Err("USB InvalidEndpoint"),
                usb_device::UsbError::InvalidState => Err("USB InvalidState"),
                usb_device::UsbError::EndpointMemoryOverflow => Err("USB EndpointMemoryOverflow"),
                usb_device::UsbError::Unsupported => Err("USB Unsupported"),
            },
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
