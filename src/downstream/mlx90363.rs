use core::{
    convert::Infallible,
    ops::{BitXor, Shl, Shr},
};

use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};

use super::{
    spi_protocol::{NegiconProtocol, NopError, NopMessage, SpiError},
    util::make_u16,
};

pub(crate) const MLXID_ADDR_LO: u16 = 0x1012u16;
pub(crate) const MLXID_ADDR_MID: u16 = 0x1014u16;
pub(crate) const MLXID_ADDR_HI: u16 = 0x1016u16;

const MEM_WRITE_KEYS: [u16; 32] = [
    17485, 31053, 57190, 57724, 7899, 53543, 26763, 12528, 38105, 51302, 16209, 24847, 13134,
    52339, 14530, 18350, 55636, 64477, 40905, 45498, 24411, 36677, 4213, 48843, 6368, 5907, 31384,
    63325, 3562, 19816, 6995, 3147,
];

#[derive(Format)]
enum MlxMemWriteStatus {
    Success = 1,
    EraseWriteFail = 2,
    EepromCrcEraseWriteFail = 4,
    KeyInvalid = 6,
    ChallengeFail = 7,
    OddAddress = 8,
}

impl MlxMemWriteStatus {
    fn from_number(number: u8) -> Self {
        match number {
            1 => Self::Success,
            2 => Self::EraseWriteFail,
            4 => Self::EepromCrcEraseWriteFail,
            6 => Self::KeyInvalid,
            7 => Self::ChallengeFail,
            8 => Self::OddAddress,
            _ => panic!("Invalid status"),
        }
    }
}

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
    MlxMemWriteChallengeReply(u16),
    MlxMemWriteReadAnswerReply(),
    MlxMemWriteStatusReply(MlxMemWriteStatus),
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
                MlxOpcode::EEWriteChallenge => Ok(MlxReply::MlxMemWriteChallengeReply(make_u16(
                    data[3], data[2],
                ))),
                MlxOpcode::EEReadAnswer => Ok(MlxReply::MlxMemWriteReadAnswerReply()),
                MlxOpcode::EEChallengeAns => Ok(MlxReply::MlxMemWriteStatusReply(
                    MlxMemWriteStatus::from_number(data[0]),
                )),
                MlxOpcode::EEWriteStatus => Ok(MlxReply::MlxMemWriteStatusReply(
                    MlxMemWriteStatus::from_number(data[0]),
                )),
                _ => {
                    warn!("Unknown opcode: {:x}", opcode as u8);
                    Err(MlxError::FormatError)
                }
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
            0x13u8 => Self::GET1,
            0x14u8 => Self::GET2,
            0x15u8 => Self::GET3,
            0x2Du8 => Self::Get3Ready,
            0x01u8 => Self::MemoryRead,
            0x02u8 => Self::MemoryReadAnswer,
            0x03u8 => Self::EEWrite,
            0x04u8 => Self::EEWriteChallenge,
            0x05u8 => Self::EEChallengeAns,
            0x28u8 => Self::EEReadAnswer,
            0x0Fu8 => Self::EEReadChallenge,
            0x0Eu8 => Self::EEWriteStatus,
            0x10u8 => Self::NOPChallenge,
            0x11u8 => Self::ChallengeNOPMISOPacket,
            0x16u8 => Self::DiagnosticDetails,
            0x17u8 => Self::DiagnosticsAnswer,
            0x18u8 => Self::OscCounterStart,
            0x19u8 => Self::OscCounterStartAcknowledge,
            0x1Au8 => Self::OscCounterStop,
            0x1Bu8 => Self::OscCounterStopAckCounterValue,
            0x2Fu8 => Self::Reboot,
            0x31u8 => Self::Standby,
            0x32u8 => Self::StandbyAck,
            0x3Du8 => Self::ErrorFrame,
            0x3Eu8 => Self::NothingToTransmit,
            0x2Cu8 => Self::ReadyMessage,
            0xFFu8 => Self::NotAnOpcode,
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

struct MlxMemWriteRequest {
    addr: u8,
    data: u16,
}

impl MlxRequest for MlxMemWriteRequest {
    fn serialize(&self) -> [u8; 8] {
        let key = MEM_WRITE_KEYS[(self.addr & 0x3e).shr(1) as usize];
        [
            0,
            self.addr,
            key as u8,
            key.shr(8) as u8,
            self.data as u8,
            self.data.shr(8) as u8,
            MlxMarker::Irregular.to_number() | MlxOpcode::EEWrite as u8,
            0,
        ]
    }
}

struct MlxMemWriteChallengeRequest {}

impl MlxRequest for MlxMemWriteChallengeRequest {
    fn serialize(&self) -> [u8; 8] {
        [
            0,
            0,
            0,
            0,
            0,
            0,
            MlxMarker::Irregular.to_number() | MlxOpcode::EEReadChallenge as u8,
            0,
        ]
    }
}

struct MlxMemWriteChallengeSolutionRequest {
    value: u16,
}

impl MlxRequest for MlxMemWriteChallengeSolutionRequest {
    fn serialize(&self) -> [u8; 8] {
        [
            0,
            0,
            (self.value as u8).bitxor(0x34),
            (self.value.shr(8) as u8).bitxor(0x12),
            (!self.value as u8).bitxor(0x34),
            !(self.value.shr(8) as u8).bitxor(0x12),
            MlxMarker::Irregular.to_number() | MlxOpcode::EEChallengeAns as u8,
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

    pub(crate) fn write_memory<D, T>(
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut (dyn OutputPin<Error = Infallible>),
        delay: &mut Delay,
        value: i16,
        addr: u8,
    ) where
        D: SpiDevice,
        T: ValidSpiPinout<D>,
    {
        delay.delay_us(150);
        let _ = Self::nop(spi, cs, 0x3939);
        delay.delay_us(150);
        let _ = Self::transfer(
            spi,
            cs,
            &MlxMemWriteRequest {
                addr: addr,
                data: value as u16,
            },
        );
        delay.delay_us(150);
        let challenge = Self::transfer(spi, cs, &MlxMemWriteChallengeRequest {});

        let chal_answer = match challenge {
            Ok(res) => match res {
                MlxReply::MlxMemWriteChallengeReply(chal) => {
                    let solution = MlxMemWriteChallengeSolutionRequest { value: chal };
                    delay.delay_us(150);
                    Self::transfer(spi, cs, &solution)
                }
                _ => {
                    return error!(
                        "Did not receive mem write challenge, got {}. Aborting write",
                        res
                    )
                }
            },
            Err(e) => return error!("Got error {}. Aborting write", e),
        };
        match chal_answer {
            Ok(res) => match res {
                MlxReply::MlxMemWriteReadAnswerReply() => delay.delay_ms(33),
                _ => return error!("Did not receive mem write challenge answer. Aborting write"),
            },
            Err(_) => return error!("Did not receive mem write challenge answer. Aborting write"),
        };
        let status = Self::nop(spi, cs, 0x3939);
        match status {
            Ok(s) => match s {
                MlxReply::MlxMemWriteStatusReply(status) => {
                    info!("Memory write completed with status: {:?}", status);
                }
                _ => error!("Failed to read status after mem write"),
            },
            Err(_) => error!("Failed to read status after mem write"),
        }
    }
}

//impl<NopMessage> Mlx90363<NopMessage> {}
