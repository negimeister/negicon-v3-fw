extern crate alloc;
use core::convert::Infallible;

use alloc::boxed::Box;
use cortex_m::delay::Delay;
use defmt::{debug, warn};
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::{
    spi::{Enabled, SpiDevice as HalSpiDevice, ValidSpiPinout},
    Spi,
};

use crate::{downstream::mlx90363::Mlx90363, negicon_event::NegiconEvent};

use super::spi_protocol::{
    NegiconProtocol, NopError, NopMessage, SpiError, NOP_REPLY_OPCODE_MLX, NOP_REPLY_OPCODE_RP,
    NOP_REPLY_OPCODE_STM,
};

pub(crate) enum DownstreamError {
    SpiError(SpiError),
    UnknownDevice(u8),
    NopError(NopError),
}

pub(crate) struct SpiDownstream<'a, P, D, T>
where
    P: OutputPin,
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    cs: &'a mut P,
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
}

impl<D, T> DownstreamDevice<D, T> for Mlx90363
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    fn poll(
        &mut self,
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
    ) -> Result<Option<NegiconEvent>, DownstreamError> {
        match self.get_alpha(spi, cs) {
            Ok(_) => todo!(),
            Err(_) => todo!(),
        }
    }
}

impl<'a, P, D, T> SpiDownstream<'a, P, D, T>
where
    P: OutputPin<Error = Infallible>,
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    pub(crate) fn new(cs: &'a mut P) -> Self {
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
            DownstreamState::Initialized(dev) => dev.as_mut().poll(spi, self.cs),
        }
    }

    fn detect(
        &mut self,
        delay: &mut Delay,
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
            Err(e) => {
                match e {
                    SpiError::TxError => debug!("TxError"),
                    SpiError::CrcError => debug!("CrcError"),
                };
                return Ok(None);
            }
        };
        buf = NopMessage::new(challenge).serialize();
        delay.delay_us(120);
        let res = spi.verified_transmit(self.cs, &mut buf);
        match res {
            Ok(_) => {}
            Err(_) => return Ok(None),
        };
        let response = NopMessage::deserialize(&buf, challenge);
        match response {
            Ok(nop) => match nop.opcode {
                NOP_REPLY_OPCODE_MLX => {
                    debug!("MLX90363 detected");
                    //TODO we should probably be reading the calibration data and directly construct an encoder or something
                    let mut mlx = Box::new(Mlx90363::new());
                    //TODO maybe we should handle actual SPI protocol errors
                    let _ = mlx.get_alpha(spi, self.cs); //This will give an error but that's ok

                    self.device = DownstreamState::Initialized(mlx);

                    delay.delay_us(120);

                    Ok(None)
                }
                NOP_REPLY_OPCODE_RP => {
                    debug!("RP2040 detected");
                    Ok(None)
                }
                NOP_REPLY_OPCODE_STM => {
                    debug!("STM32 detected");
                    Ok(None)
                }
                _ => Err(DownstreamError::UnknownDevice(nop.opcode)),
            },
            Err(e) => match e {
                NopError::InvalidOpcode(m) => {
                    warn!("Weird downstream behavior {}", m);
                    Err(DownstreamError::NopError(e))
                }
                NopError::InvalidChallenge(m) => {
                    warn!("Weird downstream behavior {}", m);
                    Err(DownstreamError::NopError(e))
                }
            },
        }
    }
}
