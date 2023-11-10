use core::{
    convert::Infallible,
    ops::{Shl, Shr},
};

use cortex_m::delay::Delay;
use defmt::{debug, error, info, warn, Format};
use embedded_hal::{digital::v2::OutputPin, prelude::_embedded_hal_blocking_spi_Transfer};
use rp_pico::hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};

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
            _ => Self::NotAnOpcode,
        }
    }
}

const CBA_256_TAB: [u8; 256] = [
    0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd, 0x57, 0x78, 0x09, 0x26, 0xeb, 0xc4, 0xb5, 0x9a,
    0xae, 0x81, 0xf0, 0xdf, 0x12, 0x3d, 0x4c, 0x63, 0xf9, 0xd6, 0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34,
    0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0, 0x91, 0xbe, 0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9,
    0xdd, 0xf2, 0x83, 0xac, 0x61, 0x4e, 0x3f, 0x10, 0x8a, 0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47,
    0xe6, 0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b, 0xb1, 0x9e, 0xef, 0xc0, 0x0d, 0x22, 0x53, 0x7c,
    0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85, 0x1f, 0x30, 0x41, 0x6e, 0xa3, 0x8c, 0xfd, 0xd2,
    0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58, 0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f,
    0x3b, 0x14, 0x65, 0x4a, 0x87, 0xa8, 0xd9, 0xf6, 0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1,
    0xe3, 0xcc, 0xbd, 0x92, 0x5f, 0x70, 0x01, 0x2e, 0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56, 0x79,
    0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80, 0x1a, 0x35, 0x44, 0x6b, 0xa6, 0x89, 0xf8, 0xd7,
    0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d, 0xc7, 0xe8, 0x99, 0xb6, 0x7b, 0x54, 0x25, 0x0a,
    0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad, 0xdc, 0xf3, 0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa, 0x8b, 0xa4,
    0x05, 0x2a, 0x5b, 0x74, 0xb9, 0x96, 0xe7, 0xc8, 0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f,
    0xab, 0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66, 0xfc, 0xd3, 0xa2, 0x8d, 0x40, 0x6f, 0x1e, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xca, 0xe5, 0x94, 0xbb, 0x21, 0x0e, 0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec,
    0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a, 0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42,
];
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

enum MlxError {
    IncorrectBitCount,
    IncorrectCrc,
    NTT,
    InvalidOpcode,
}

impl MlxError {
    fn from_number(number: u8) -> Self {
        match number {
            1 => Self::IncorrectBitCount,
            2 => Self::IncorrectCrc,
            3 => Self::NTT,
            4 => Self::InvalidOpcode,
            _ => panic!("Invalid error code"),
        }
    }
}

pub(crate) struct MlxAlpha {
    pub data: u16,
    pub diag: MlxDiagnosticStatus,
    pub vg: u8,
    pub counter: u8,
}

impl MlxAlpha {
    fn from_message(message: &[u8; 8]) -> Result<Self, &'static str> {
        if message[6] & 0xC0 != 0 {
            return Err("Not an alpha message");
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
    crc: u8,
}

impl MlxFrame {
    fn from_message(message: &[u8; 8]) -> Self {
        Self {
            marker: MlxMarker::from_number(message[6] >> 6),
            opcode: MlxOpcode::from_number(message[6] & 0x3F),
            crc: message[7],
        }
    }
}

pub(crate) struct MlxStatus {
    fw_version: u8,
    hw_version: u8,
}

impl MlxStatus {
    fn from_message(message: &[u8; 8]) -> Result<Self, &'static str> {
        if message[6] != (MlxMarker::Irregular.to_number().shl(6) | MlxOpcode::ReadyMessage as u8) {
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

struct MlxNopChallenge {
    key: u16,
}

impl MlxNopChallenge {
    fn encode(&self) -> [u8; 8] {
        let data: [u8; 8] = [
            0,
            0,
            self.key as u8,
            self.key.shr(8) as u8, //TODO check if timeout should be adjusted
            0,
            0,
            (MlxMarker::Irregular as u8).shl(6u8) | MlxOpcode::NOPChallenge as u8,
            0,
        ];
        data
    }
}

struct MlxNopResponse {}

impl MlxNopResponse {
    fn verify(data: &[u8; 8], challenge: u16) -> Result<(), &'static str> {
        let frame = MlxFrame::from_message(data);
        let key = data[2] as u16 | (data[3] as u16).shl(8);
        let inverted_key: u16 = data[4] as u16 | (data[5] as u16).shl(8);
        match frame.opcode {
            MlxOpcode::ChallengeNOPMISOPacket => {}
            _ => return Err("Invalid NOP response opcode"),
        }
        if key != challenge {
            return Err("Invalid NOP response key");
        }
        if key != !inverted_key {
            return Err("Invalid NOP response inverted key");
        }
        Ok(())
    }
}

pub(crate) struct Mlx90363 {}

impl Mlx90363 {
    pub(crate) fn new() -> Mlx90363 {
        Mlx90363 {}
    }

    fn check_message(data: &[u8; 8]) -> Result<(), &'static str> {
        match Self::verify_crc(data) {
            Ok(_) => {}
            Err(e) => return Err(e),
        };

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
                MlxOpcode::ErrorFrame => match MlxError::from_number(data[0]) {
                    MlxError::IncorrectBitCount => Err("Incorrect bit count"),
                    MlxError::IncorrectCrc => Err("Incorrect CRC"),
                    MlxError::NTT => Err("Answer timeout or not ready"),
                    MlxError::InvalidOpcode => Err("Invalid opcode"),
                },
                MlxOpcode::NothingToTransmit => {
                    info!("Nothing to transmit");
                    Ok(())
                }
                MlxOpcode::ChallengeNOPMISOPacket => {
                    info!("Challenge NOP MISO packet");
                    Ok(())
                }
                MlxOpcode::NotAnOpcode => Err("Invalid opcode"),
                _ => Err("Unknown irregular frame"),
            },
        }
    }

    fn crc(data: &[u8]) -> u8 {
        let mut crc: u8 = 0xFF;
        crc = CBA_256_TAB[(crc ^ data[0]) as usize];
        crc = CBA_256_TAB[(crc ^ data[1]) as usize];
        crc = CBA_256_TAB[(crc ^ data[2]) as usize];
        crc = CBA_256_TAB[(crc ^ data[3]) as usize];
        crc = CBA_256_TAB[(crc ^ data[4]) as usize];
        crc = CBA_256_TAB[(crc ^ data[5]) as usize];
        crc = CBA_256_TAB[(crc ^ data[6]) as usize];
        !crc
    }
    fn set_crc(data: &mut [u8]) {
        data[7] = Self::crc(data);
    }

    fn verify_crc(data: &[u8]) -> Result<(), &'static str> {
        if data.len() != 8 {
            panic!("data.len must be 8");
        }
        let checksum = Self::crc(data);
        if data[7] == checksum {
            Ok(())
        } else {
            debug!("expected {:x}", checksum);
            Err("CRC Error")
        }
    }

    fn transfer<D>(
        spi: &mut Spi<Enabled, D, impl ValidSpiPinout<D>, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        tx: &mut [u8; 8],
    ) -> Result<(), &'static str>
    where
        D: SpiDevice,
    {
        if tx.len() != 8 {
            panic!("tx.len must be 8");
        }
        Self::set_crc(tx);
        cs.set_low();
        match spi.transfer(tx) {
            Ok(_) => {}
            Err(_) => {
                return Err("SPI error");
            }
        };
        cs.set_high();
        Self::check_message(tx)
    }

    pub(crate) fn get_alpha(&mut self) -> Result<MlxAlpha, &'static str> {
        let mut data: [u8; 8] = MlxGET1 {
            reset_counter: false,
            timeout: 0xffff,
            marker: MlxMarker::Alpha,
        }
        .encode();
        match self.transfer(&mut data) {
            Ok(_) => match MlxAlpha::from_message(&data) {
                Ok(alpha) => Ok(alpha),
                Err(e) => Err(e),
            },
            Err(e) => Err(e),
        }
    }

    pub(crate) fn init(&mut self, delay: &mut Delay) -> Result<(), &'static str> {
        delay.delay_ms(100);
        let challenge = 0x5A5A;
        loop {
            let mut nop = MlxNopChallenge { key: challenge }.encode();
            match self.transfer(&mut nop) {
                Ok(_) => match MlxStatus::from_message(&nop) {
                    Ok(status) => {
                        info!(
                            "Status: hw: {}, fw: {}",
                            status.hw_version, status.fw_version
                        );
                    }
                    Err(_e) => match MlxNopResponse::verify(&nop, challenge) {
                        Ok(_) => {
                            info!("NOP response verified");
                            delay.delay_us(200);
                            break;
                        }
                        Err(e) => {
                            warn!("Failed to verify NOP response\n{:?}\nTry again", e);
                        }
                    },
                },
                Err(e) => {
                    error!("Received error on startup\n{:?}\nContinuing anyway", e);
                }
            }

            delay.delay_us(200);
        }
        let mut get1: [u8; 8] = MlxGET1 {
            reset_counter: false,
            timeout: 0xffff,
            marker: MlxMarker::Alpha,
        }
        .encode();
        match self.transfer(&mut get1) {
            Ok(_) => match MlxNopResponse::verify(&get1, challenge) {
                Ok(_) => Ok(delay.delay_ms(1)),
                Err(e) => return Err(e),
            },
            Err(e) => Err(e),
        }
    }
}
