extern crate alloc;
use alloc::boxed::Box;
use cortex_m::delay::Delay;
use defmt::{debug, warn};
use embedded_hal::{digital::v2::OutputPin, prelude::_embedded_hal_blocking_spi_Transfer};
use rp_pico::hal::{
    spi::{Enabled, SpiDevice as HalSpiDevice, ValidSpiPinout},
    Spi,
};

use crate::negicon_event::NegiconEvent;

use super::spi_protocol::{
    NopError, NopMessage, NOP_REPLY_OPCODE_MLX, NOP_REPLY_OPCODE_RP, NOP_REPLY_OPCODE_STM,
};

struct SpiDownstream<'a, P, D, T>
where
    P: OutputPin,
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    cs: &'a mut P,
    device: DownstreamState<D, T>,
}

enum DownstreamState<D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    Uninitialized,
    Initialized(Box<dyn DownstreamDevice<D, T>>),
}
trait DownstreamDevice<D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    fn poll(&mut self, spi: &mut Spi<Enabled, D, T, 8>) -> Result<Option<NegiconEvent>, ()>;
}

impl<'a, P, D, T> SpiDownstream<'a, P, D, T>
where
    P: OutputPin<Error = ()>,
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    fn new(cs: &'a mut P) -> Self {
        Self {
            cs,
            device: DownstreamState::Uninitialized,
        }
    }

    fn detect(
        &mut self,
        delay: &mut Delay,
        spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<Option<&dyn DownstreamDevice<D, T>>, &'static str>
    where
        D: HalSpiDevice,
        T: ValidSpiPinout<D>,
    {
        let challenge = 0x3939;
        self.cs.set_low().unwrap();
        let mut buf = NopMessage::new(challenge).serialize();
        spi.transfer(&mut buf).unwrap();
        self.cs.set_high().unwrap();
        delay.delay_us(5);
        self.cs.set_low().unwrap();
        spi.transfer(&mut buf).unwrap();
        self.cs.set_high().unwrap();
        let response = NopMessage::deserialize(&buf, challenge);
        match response {
            Ok(nop) => match nop.opcode {
                NOP_REPLY_OPCODE_MLX => {
                    debug!("MLX90363 detected");
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
                _ => Err("Unknown device detected"),
            },
            Err(e) => match e {
                NopError::InvalidOpcode(m) => {
                    warn!("Weird downstream behavior {}", m);
                    Err(m)
                }
                NopError::InvalidCrc(_) => return Ok(None),
                NopError::InvalidChallenge(m) => {
                    warn!("Weird downstream behavior {}", m);
                    Err(m)
                }
            },
        }
    }
}
