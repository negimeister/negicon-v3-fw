use core::cmp::min;




use super::mlx90363::Mlx90363;

pub(crate) struct NegiconEncoder {
    sensor: Mlx90363,
    min: u16,
    max: u16,
    last: u16,
    deadzone: u16,
}

impl NegiconEncoder {
    pub(crate) fn new(sensor: Mlx90363, min: u16, max: u16, last: u16, deadzone: u16) -> Self {
        Self {
            sensor,
            min,
            max,
            last,
            deadzone,
        }
    }

    pub(crate) fn calculate_output(&mut self, alpha: u16) -> i16 {
        let diff = min(alpha - self.last, alpha + self.max - self.last);
        if diff < self.deadzone {
            0
        } else {
            diff as i16
        }
    }
}
