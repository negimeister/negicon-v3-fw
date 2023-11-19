extern crate alloc;
use core::convert::Infallible;

use alloc::boxed::Box;
use cortex_m::delay::Delay;
use defmt::{debug, error, info, warn, Format};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    spi::{Enabled, SpiDevice as HalSpiDevice, ValidSpiPinout},
    Spi,
};

use crate::{downstream::mlx_downstream::MlxDownstream, negicon_event::NegiconEvent};

use super::{
    mlx90363::MlxError,
    spi_protocol::{
        NegiconProtocol, NopError, NopMessage, SpiError, NOP_REPLY_OPCODE_MLX, NOP_REPLY_OPCODE_RP,
        NOP_REPLY_OPCODE_STM,
    },
};
#[derive(Format)]
pub(crate) enum DownstreamError {
    SpiError(SpiError),
    UnknownDevice(u8),
    NopError(NopError),
    MlxError(MlxError),
    UnexpectedReply,
}

pub(crate) struct SpiDownstream<'a, D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    cs: &'a mut dyn OutputPin<Error = Infallible>,
    pub(crate) device: DownstreamState<D, T>,
}

pub(crate) enum DownstreamState<D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    Uninitialized,
    Initialized(Box<dyn DownstreamDevice<D, T>>),
}
pub(crate) trait DownstreamDevice<D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    fn poll(
        &mut self,
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
    ) -> Result<Option<NegiconEvent>, DownstreamError>;

    fn write_memory(
        &mut self,
        _spi: &mut Spi<Enabled, D, T, 8>,
        _cs: &mut dyn OutputPin<Error = Infallible>,
        _delay: &mut Delay,
        _writeEvent: &NegiconEvent,
    ) {
        error!("Memory write target not implemented");
    }
}

impl<'a, D, T> SpiDownstream<'a, D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    pub(crate) fn new(cs: &'a mut dyn OutputPin<Error = Infallible>) -> Self {
        Self {
            cs,
            device: DownstreamState::Uninitialized,
        }
    }

    pub fn poll(
        &mut self,
        delay: &mut Delay,
        spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<Option<NegiconEvent>, DownstreamError> {
        match &mut self.device {
            DownstreamState::Uninitialized => self.detect(delay, spi),
            DownstreamState::Initialized(dev) => match dev.as_mut().poll(spi, self.cs) {
                Ok(event) => Ok(event),
                Err(e) => match e {
                    DownstreamError::SpiError(_) => {
                        self.device = DownstreamState::Uninitialized;
                        info!("SPI Error, removing downstream");
                        Ok(None)
                    }
                    DownstreamError::MlxError(_) => {
                        self.device = DownstreamState::Uninitialized;
                        info!("MLX Error, removing downstream");
                        Ok(None)
                    }
                    _ => Err(e),
                },
            },
        }
    }

    pub(crate) fn write_memory(
        &mut self,
        write_event: &NegiconEvent,
        spi: &mut Spi<Enabled, D, T, 8>,
        delay: &mut Delay,
    ) {
        info!(
            "Downstream memory write request. Id: {}, Address: {:x}, Value: {:x}",
            write_event.id, write_event.sequence, write_event.value
        );
        match &mut self.device {
            DownstreamState::Uninitialized => {
                error!("Memory write target not inialized")
            }
            DownstreamState::Initialized(dev) => {
                dev.as_mut().write_memory(spi, self.cs, delay, write_event);
            }
        }
    }

    fn detect(
        &mut self,
        _delay: &mut Delay,
        spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<Option<NegiconEvent>, DownstreamError>
    where
        D: HalSpiDevice,
        T: ValidSpiPinout<D>,
    {
        let challenge = 0x3939;
        let mut buf = NopMessage::new(challenge).serialize();
        let res = spi.verified_transmit(self.cs, &mut buf);
        match res {
            Ok(_) => {}
            Err(_) => {
                return Ok(None);
            }
        };
        let response = NopMessage::deserialize(&buf);

        match response {
            Ok(nop) => match nop.verify(challenge) {
                Ok(_) => match nop.opcode {
                    NOP_REPLY_OPCODE_MLX => {
                        info!("MLX90363 detected");
                        self.device = DownstreamState::Initialized(Box::new(MlxDownstream::new()));
                        Ok(None)
                    }
                    NOP_REPLY_OPCODE_RP => {
                        info!("RP2040 detected");
                        Ok(None)
                    }
                    NOP_REPLY_OPCODE_STM => {
                        info!("STM32 detected");
                        Ok(None)
                    }
                    _ => Err(DownstreamError::UnknownDevice(nop.opcode)),
                },
                Err(e) => {
                    warn!("Invalid challenge response: {:?}", e);
                    return Ok(None);
                }
            },
            Err(e) => match e {
                NopError::InvalidOpcode(_m) => Err(DownstreamError::NopError(e)),
                NopError::InvalidChallenge(m) => {
                    warn!("Weird downstream behavior {}", m);
                    Err(DownstreamError::NopError(e))
                }
            },
        }
    }
}
