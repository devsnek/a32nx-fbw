use crate::Result;

#[derive(PartialEq)]
pub(crate) enum PitchControlMode {
    Ground,
    Flight,
    Flare,
}

pub(crate) struct PitchControl {
    pub(crate) mode: PitchControlMode,
}
impl Default for PitchControl {
    fn default() -> Self {
        PitchControl {
            mode: PitchControlMode::Ground,
        }
    }
}

impl PitchControl {
    pub(crate) fn update(&mut self) -> Result<()> {
        match self.mode {
            PitchControlMode::Ground => self.handle_ground_transitions(),
            PitchControlMode::Flight => self.handle_flight_transitions(),
            PitchControlMode::Flare => self.handle_flare_transitions(),
        }

        Ok(())
    }

    fn handle_ground_transitions(&mut self) {
        /*
        if radio_altimeter > 50 || (in_flight && pitch_attitude > 8.0) {

        } else {

        }
         */
    }

    fn handle_flight_transitions(&mut self) {}

    fn handle_flare_transitions(&mut self) {}
}
