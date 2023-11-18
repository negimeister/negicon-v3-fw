use crate::downstream::util::{make_i16, make_u16};
use core::ops::Shr;

pub(crate) struct NegiconEvent {
    pub(crate) event_type: NegiconEventType,
    pub(crate) id: u16,
    pub(crate) value: i16,
    pub(crate) controller_id: u8,
    pub(crate) sequence: u8,
}

#[derive(PartialEq, Clone, Copy)]
pub(crate) enum NegiconEventType {
    Input,
    Output,
    MemWrite,
    Reboot,
}

impl NegiconEvent {
    pub(crate) fn new(
        event_type: NegiconEventType,
        id: u16,
        value: i16,
        controller_id: u8,
        sequence: u8,
    ) -> Self {
        NegiconEvent {
            event_type,
            id,
            value,
            controller_id,
            sequence,
        }
    }

    pub(crate) fn serialize(&self) -> [u8; 8] {
        [
            self.event_type as u8,
            self.id.shr(8) as u8,
            self.id as u8,
            self.value.shr(8) as u8,
            self.value as u8,
            self.controller_id,
            self.sequence,
            0u8,
        ]
    }

    pub(crate) fn deserialize(data: [u8; 8]) -> Self {
        let event_type = match data[0] {
            0 => NegiconEventType::Input,
            1 => NegiconEventType::Output,
            2 => NegiconEventType::MemWrite,
            3 => NegiconEventType::Reboot,
            _ => NegiconEventType::Input,
        };
        let id = make_u16(data[1], data[2]);
        let value = make_i16(data[3], data[4]);
        let controller_id = data[5];
        let sequence = data[6];
        NegiconEvent {
            event_type,
            id,
            value,
            controller_id,
            sequence,
        }
    }
}
