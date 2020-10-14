use crate::{
    clamp,
    data::Data,
    input::Input,
    linear_range,
    pid::PIDController,
    pitch_control::{PitchControl, PitchControlMode},
    protections::NormalLawProtections,
    sim_time::SimTime,
    Result,
};
use msfs::{
    sim_connect::{data_definition, SimConnect},
    sys::SIMCONNECT_OBJECT_ID_USER,
};

const CONTROL_SURFACES: u32 = 0;

#[data_definition]
#[derive(Default)]
struct ControlSurfaces {
    #[name = "ELEVATOR POSITION"]
    #[unit = "Position"]
    elevator: f64,
    #[name = "AILERON POSITION"]
    #[unit = "Position"]
    ailerons: f64,
    #[name = "RUDDER POSITION"]
    #[unit = "Position"]
    rudder: f64,
}

#[derive(Default)]
pub(crate) struct Controls {
    surfaces: ControlSurfaces,
    pitch_controller: PitchController,
    roll_controller: RollController,
}

impl Controls {
    pub(crate) fn init(&self, sim: &SimConnect) -> Result<()> {
        sim.add_data_definition::<ControlSurfaces>(CONTROL_SURFACES)?;

        Ok(())
    }

    pub(crate) fn update(
        &mut self,
        sim: &SimConnect,
        sim_time: &SimTime,
        data: &Data,
        protections: &NormalLawProtections,
        pitch_control: &PitchControl,
        input: &Input,
    ) -> Result<()> {
        if data.autopilot() {
            self.surfaces.elevator = input.yoke_y;
            self.surfaces.ailerons = input.yoke_x;
            self.surfaces.rudder = input.rudder;
        } else {
            self.surfaces.elevator = self.pitch_controller.calculate(
                self.surfaces.elevator,
                sim_time,
                data,
                protections,
                pitch_control,
                input,
            );
            self.surfaces.ailerons = self.roll_controller.calculate(
                self.surfaces.ailerons,
                sim_time,
                data,
                protections,
                pitch_control,
                input,
            );
            self.surfaces.rudder = input.rudder; // TODO: yaw FBW
        }

        sim.set_data_on_sim_object(CONTROL_SURFACES, SIMCONNECT_OBJECT_ID_USER, &self.surfaces)?;

        Ok(())
    }
}

struct PitchController {
    aoa_controller: PIDController,
}
impl Default for PitchController {
    fn default() -> Self {
        PitchController {
            // AoA error -> elevator handle movement rate
            aoa_controller: PIDController::new(-2.0, 2.0, 0.002, 0.0, 0.0002),
        }
    }
}

impl PitchController {
    fn calculate(
        &mut self,
        elevator: f64,
        sim_time: &SimTime,
        data: &Data,
        protections: &NormalLawProtections,
        pitch_control: &PitchControl,
        input: &Input,
    ) -> f64 {
        // On the ground, pitch is direct
        // TODO: Add ground mode calculations (e.g. when aircraft reaches 70 knots during the T/O roll, maximum deflection of elevators is affected)
        let new_elevator = if pitch_control.mode == PitchControlMode::Ground {
            input.yoke_y
        } else if protections.aoa_demand_active {
            // AoA protections are available in both flight/flare modes
            elevator + {
                let dt = sim_time.delta();
                let commanded_aoa = if input.yoke_y >= 0.0 {
                    // Neutral -> Full Up = AoA proportional range from alpha_prot -> alpha_max
                    linear_range(input.yoke_y, data.alpha_prot(), data.alpha_max())
                } else {
                    // Neutral -> Full Down = AoA proportional range from alpha_prot -> 0 AoA
                    linear_range(input.yoke_y, data.alpha_prot(), 0.0)
                };
                let mut delta_elevator =
                    self.aoa_controller.update(commanded_aoa - data.alpha(), dt);

                // Apply protections
                delta_elevator = self.load_factor_limitation(delta_elevator);
                // This isn't specified in the FCOM, but the flight model is not true enough to real life.
                delta_elevator = self.pitch_attitude_protection(delta_elevator);

                delta_elevator
            }
        } else if pitch_control.flare_effect > 0.0 {
            // Flare mode has a special effect and does not have all the protections of flight mode
            0.0
        } else {
            // Flight mode
            0.0
        };

        clamp(new_elevator, -1.0, 1.0)
    }
}

struct RollController {
    roll: f64, // The desired bank angle
    controller: PIDController,
}
impl Default for RollController {
    fn default() -> Self {
        RollController {
            roll: 0.0,
            controller: PIDController::new(-1.0, 1.0, 0.10, 0.0, 0.02),
        }
    }
}

impl RollController {
    fn calculate(
        &mut self,
        ailerons: f64,
        sim_time: &SimTime,
        data: &Data,
        protections: &NormalLawProtections,
        pitch_control: &PitchControl,
        input: &Input,
    ) -> f64 {
        // TODO: Handle other control laws besides normal law
        let dt = sim_time.delta();
        match pitch_control.mode {
            PitchControlMode::Flight | PitchControlMode::Flare => {
                if input.yoke_x == 0.0 {
                    // If we are banked beyond the nominal bank angle, roll back to the nominal bank angle
                    if self.roll.abs() > protections.nominal_bank_angle {
                        self.roll += 5.0 * -self.roll.signum() * dt;
                        if self.roll.abs() < protections.nominal_bank_angle {
                            self.roll = self.roll.signum() * protections.nominal_bank_angle;
                        }
                    }
                // We should be holding the specified roll angle
                } else {
                    // We should be responsive to the user's roll request
                    self.roll += 15.0 * input.yoke_x * dt; // 15 degrees/sec at maximum deflection
                    self.roll = clamp(self.roll, 0.0, 1.0);
                }
                self.controller.update(self.roll - data.roll(), dt)
            }
            _ => ailerons,
        }
    }
}
