use core::ops::Shr;

pub(crate) struct NegiconEvent {
    pub(crate) id: u16,
    pub(crate) value: i16,
    pub(crate) controller_id: u8,
    pub(crate) sequence: u8,
}

impl NegiconEvent {
    pub(crate) fn new(id: u16, value: i16, controller_id: u8, sequence: u8) -> Self {
        NegiconEvent {
            id,
            value,
            controller_id,
            sequence,
        }
    }

    pub(crate) fn serialize(&self) -> [u8; 8] {
        [
            self.id.shr(8) as u8,
            self.id as u8,
            self.value.shr(8) as u8,
            self.value as u8,
            self.controller_id,
            self.sequence,
            0u8,
            0u8,
        ]
    }
}
