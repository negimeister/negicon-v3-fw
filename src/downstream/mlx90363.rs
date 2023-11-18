use core::{
    convert::Infallible,
    fmt::Error,
    marker::PhantomData,
    ops::{Shl, Shr},
};

use defmt::{debug, info, Format};
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};
use usb_device::endpoint::In;

use super::{
    spi_protocol::{NegiconProtocol, NopError, NopMessage, SpiError},
    util::make_u16,
};

pub(crate) const MLXID_ADDR_LO: u16 = 0x1012u16;
pub(crate) const MLXID_ADDR_MID: u16 = 0x1014u16;
pub(crate) const MLXID_ADDR_HI: u16 = 0x1016u16;

pub(crate) trait MlxRequest {
    fn serialize(&self) -> [u8; 8];
}

impl MlxRequest for MlxGET1 {
    fn serialize(&self) -> [u8; 8] {
        MlxGET1::encode(&self)
    }
}

impl MlxRequest for MlxMemReadRequest {
    fn serialize(&self) -> [u8; 8] {
        MlxMemReadRequest::serialize(&self)
    }
}

impl MlxRequest for NopMessage {
    fn serialize(&self) -> [u8; 8] {
        NopMessage::serialize(&self)
    }
}

#[derive(Format)]
pub(crate) enum MlxReply {
    Nop(NopMessage),
    MlxAlpha(MlxAlpha),
    MlxMemReadResponse(MlxMemReadResponse),
    XReply(),
}

impl MlxReply {
    pub(crate) fn deserialize(data: [u8; 8]) -> Result<Self, MlxError> {
        let frame = MlxFrame::from_message(&data);
        let opcode = frame.opcode;
        match frame.marker {
            MlxMarker::Alpha => MlxAlpha::from_message(&data).map(|a| MlxReply::MlxAlpha(a)),
            MlxMarker::AlphaBeta => todo!(),
            MlxMarker::XYZ => todo!(),
            MlxMarker::Irregular => match opcode {
                MlxOpcode::ReadyMessage => Ok(MlxReply::XReply()),
                MlxOpcode::ErrorFrame => {
                    Err(MlxError::DeviceError(DeviceError::from_number(data[0])))
                }
                MlxOpcode::NothingToTransmit => {
                    info!("Nothing to transmit");
                    Ok(MlxReply::XReply())
                }
                MlxOpcode::ChallengeNOPMISOPacket => {
                    match NopMessage::deserialize(&data).map(|n| MlxReply::Nop(n)) {
                        Ok(nop) => Ok(nop),
                        Err(e) => Err(MlxError::NopError(e)),
                    }
                }
                MlxOpcode::NotAnOpcode => Err(MlxError::DeviceError(
                    DeviceError::InvalidResponseOpcode(opcode as u8),
                )),
                MlxOpcode::MemoryReadAnswer => Ok(MlxReply::MlxMemReadResponse(
                    MlxMemReadResponse::deserialize(&data),
                )),
                _ => Err(MlxError::FormatError),
            },
        }
    }
}

#[allow(dead_code)]
pub enum MlxOpcode {
    GET1 = 0x13,
    GET2 = 0x14,
    GET3 = 0x15,
    Get3Ready = 0x2D,
    MemoryRead = 0x01,
    MemoryReadAnswer = 0x02,
    EEWrite = 0x03,
    EEWriteChallenge = 0x04,
    EEChallengeAns = 0x05,
    EEReadAnswer = 0x28,
    EEReadChallenge = 0x0F,
    EEWriteStatus = 0x0E,
    NOPChallenge = 0x10,
    ChallengeNOPMISOPacket = 0x11,
    DiagnosticDetails = 0x16,
    DiagnosticsAnswer = 0x17,
    OscCounterStart = 0x18,
    OscCounterStartAcknowledge = 0x19,
    OscCounterStop = 0x1A,
    OscCounterStopAckCounterValue = 0x1B,
    Reboot = 0x2F,
    Standby = 0x31,
    StandbyAck = 0x32,
    ErrorFrame = 0x3D,
    NothingToTransmit = 0x3E,
    ReadyMessage = 0x2C,
    NotAnOpcode = 0xFF,
}

impl MlxOpcode {
    fn from_number(number: u8) -> Self {
        match number {
            0x2Cu8 => Self::ReadyMessage,
            0x3Du8 => Self::ErrorFrame,
            0x3Eu8 => Self::NothingToTransmit,
            0x11u8 => Self::ChallengeNOPMISOPacket,
            0x02u8 => Self::MemoryReadAnswer,
            _ => Self::NotAnOpcode,
        }
    }
}

#[derive(Format)]
pub(crate) enum MlxDiagnosticStatus {
    Pending,
    Fail,
    Pass,
    NewCycle,
}

impl MlxDiagnosticStatus {
    pub(crate) fn from_number(number: u8) -> Self {
        match number {
            0 => Self::Pending,
            1 => Self::Fail,
            2 => Self::Pass,
            3 => Self::NewCycle,
            _ => panic!("Invalid diagnostic status"),
        }
    }
}

enum MlxMarker {
    Alpha,
    AlphaBeta,
    XYZ,
    Irregular,
}

impl MlxMarker {
    fn from_number(number: u8) -> Self {
        match number {
            0 => Self::Alpha,
            1 => Self::AlphaBeta,
            2 => Self::XYZ,
            3 => Self::Irregular,
            _ => panic!("Invalid marker"),
        }
    }
    fn to_number(&self) -> u8 {
        match self {
            Self::Alpha => 0,
            Self::AlphaBeta => 1u8.shl(6u8),
            Self::XYZ => 2u8.shl(6u8),
            Self::Irregular => 3u8.shl(6u8),
        }
    }
}
#[derive(Format)]
pub(crate) enum DeviceError {
    IncorrectBitCount,
    IncorrectCrc,
    NTT,
    InvalidResponseOpcode(u8),
    InvalidRequestOpcode,
    Unknown,
}

impl DeviceError {
    fn from_number(number: u8) -> Self {
        match number {
            1 => Self::IncorrectBitCount,
            2 => Self::IncorrectCrc,
            3 => Self::NTT,
            4 => Self::InvalidRequestOpcode,
            _ => Self::Unknown,
        }
    }
}
#[derive(Format)]
pub(crate) enum MlxError {
    DeviceError(DeviceError),
    SpiError(SpiError),
    FormatError,
    NopError(NopError),
}
#[allow(dead_code)]
#[derive(Format)]
pub(crate) struct MlxAlpha {
    pub data: u16,
    pub diag: MlxDiagnosticStatus,
    pub vg: u8,
    pub counter: u8,
}

impl MlxAlpha {
    pub(crate) fn from_message(message: &[u8; 8]) -> Result<Self, MlxError> {
        if message[6] & 0xC0 != 0 {
            return Err(MlxError::FormatError);
        }
        Ok(Self {
            data: message[0] as u16 | (message[1] as u16 & 0x3F).shl(8),
            diag: MlxDiagnosticStatus::from_number(message[1].shr(6)),
            vg: message[4],
            counter: message[6] & 0x3F,
        })
    }
}

struct MlxFrame {
    marker: MlxMarker,
    opcode: MlxOpcode,
}

impl MlxFrame {
    pub(crate) fn from_message(message: &[u8; 8]) -> Self {
        Self {
            marker: MlxMarker::from_number(message[6] >> 6),
            opcode: MlxOpcode::from_number(message[6] & 0x3F),
        }
    }
}
#[allow(dead_code)]
pub(crate) struct MlxStatus {
    fw_version: u8,
    hw_version: u8,
}

#[allow(dead_code)]
impl MlxStatus {
    fn from_message(message: &[u8; 8]) -> Result<Self, &'static str> {
        if message[6] != (MlxMarker::Irregular.to_number() | MlxOpcode::ReadyMessage as u8) {
            Err("Invalid Ready packed magic")
        } else {
            Ok(Self {
                fw_version: message[1],
                hw_version: message[0],
            })
        }
    }
}

struct MlxGET1 {
    reset_counter: bool,
    timeout: u16,
    marker: MlxMarker,
}

impl MlxGET1 {
    fn encode(&self) -> [u8; 8] {
        let data: [u8; 8] = [
            0,
            if self.reset_counter { 1 } else { 0 },
            self.timeout as u8,
            self.timeout.shr(8) as u8, //TODO check if timeout should be adjusted
            0,
            0,
            (self.marker.to_number()) | MlxOpcode::GET1 as u8,
            0,
        ];
        data
    }
}

struct MlxMemReadRequest {
    addr0: u16,
    addr1: u16,
}

impl MlxMemReadRequest {
    pub(crate) fn new(addr0: u16, addr1: u16) -> Self {
        Self { addr0, addr1 }
    }
    pub(crate) fn serialize(&self) -> [u8; 8] {
        [
            self.addr0 as u8,
            self.addr0.shr(8) as u8,
            self.addr1 as u8,
            self.addr1.shr(8) as u8,
            0,
            0,
            MlxMarker::Irregular.to_number() | MlxOpcode::MemoryRead as u8,
            0,
        ]
    }
}

#[derive(Format)]
pub(crate) struct MlxMemReadResponse {
    pub(crate) data0: u16,
    pub(crate) data1: u16,
}

impl MlxMemReadResponse {
    pub(crate) fn deserialize(data: &[u8; 8]) -> Self {
        Self {
            data0: make_u16(data[1], data[0]),
            data1: make_u16(data[3], data[2]),
        }
    }
}

pub(crate) struct Mlx90363 {}

/*trait MlxReadMem<R: MlxReply> {
    fn read_memory<D: SpiDevice>(
        spi: &mut Spi<Enabled, D, impl ValidSpiPinout<D>, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        3addr0: u16,
        addr1: u16,
    ) -> (R, Mlx90363<MlxMemReadResponse>) {
    }
}

impl MlxReadMem<NopMessage> for Mlx90363<NopMessage> {}*/

/*pub(crate) struct MlxStruct<R: MlxReply> {
    phantom: PhantomData<R>,
}*/

//pub(crate) trait MlxTransmit<R: MlxReply> {}

//impl<R: MlxReply> MlxTransmit<R> for MlxStruct<R> {}
pub(crate) struct XReply {}

/*impl MlxReply for XReply {
    fn deserialize(data: &[u8; 8]) -> Result<Self, MlxError> {
        Err(MlxError::FormatError)
    }
}*/

/*impl MlxStruct<XReply> {
    pub(crate) fn new() -> Self {
        Self {
            phantom: PhantomData,
        }
    }
}*/

//trait MlxBase<R: MlxReply, D: SpiDevice> {}

//pub(crate) trait MlxNop<R: MlxReply>: MlxTransmit<R> {}

/*impl MlxNop<NopMessage> for MlxStruct<NopMessage> {}
impl MlxNop<MlxAlpha> for MlxStruct<MlxAlpha> {}
impl MlxNop<XReply> for MlxStruct<XReply> {}*/

//trait MlxGet1<R: MlxReply, D: SpiDevice>: MlxBase<R, D> {}

impl Mlx90363 {
    pub(crate) fn nop<D: SpiDevice>(
        spi: &mut Spi<Enabled, D, impl ValidSpiPinout<D>, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        challenge: u16,
    ) -> Result<MlxReply, MlxError> {
        Self::transfer(spi, cs, &NopMessage::new(challenge))
    }

    fn check_message(data: &[u8; 8]) -> Result<(), MlxError> {
        let frame = MlxFrame::from_message(data);
        match frame.marker {
            MlxMarker::Alpha => Ok(()),
            MlxMarker::AlphaBeta => todo!(),
            MlxMarker::XYZ => todo!(),
            MlxMarker::Irregular => match frame.opcode {
                MlxOpcode::ReadyMessage => {
                    info!("Ready message");
                    Ok(())
                }
                MlxOpcode::ErrorFrame => {
                    Err(MlxError::DeviceError(DeviceError::from_number(data[0])))
                }
                MlxOpcode::NothingToTransmit => {
                    info!("Nothing to transmit");
                    Ok(())
                }
                MlxOpcode::ChallengeNOPMISOPacket => {
                    info!("Challenge NOP MISO packet");
                    Ok(())
                }
                MlxOpcode::NotAnOpcode => Err(MlxError::DeviceError(
                    DeviceError::InvalidResponseOpcode(frame.opcode as u8),
                )),
                _ => Err(MlxError::FormatError),
            },
        }
    }

    pub(crate) fn read_memory<D: SpiDevice>(
        spi: &mut Spi<Enabled, D, impl ValidSpiPinout<D>, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        addr0: u16,
        addr1: u16,
    ) -> Result<MlxReply, MlxError> {
        let req = MlxMemReadRequest::new(addr0, addr1);
        Self::transfer(spi, cs, &req)
    }

    fn transfer<D>(
        spi: &mut Spi<Enabled, D, impl ValidSpiPinout<D>, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        request: &dyn MlxRequest,
    ) -> Result<MlxReply, MlxError>
    where
        D: SpiDevice,
    {
        let mut buf = request.serialize();
        match spi.verified_transmit(cs, &mut buf) {
            Ok(_) => MlxReply::deserialize(buf),
            Err(e) => Err(MlxError::SpiError(e)),
        }
    }

    pub(crate) fn get_alpha<D>(
        spi: &mut Spi<Enabled, D, impl ValidSpiPinout<D>, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
    ) -> Result<MlxReply, MlxError>
    where
        D: SpiDevice,
    {
        let req = MlxGET1 {
            reset_counter: false,
            timeout: 0xffff,
            marker: MlxMarker::Alpha,
        };
        Self::transfer(spi, cs, &req)
    }
}

//impl<NopMessage> Mlx90363<NopMessage> {}
