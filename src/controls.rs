use crate::{
    clamp, fbw::FBW, linear_range, pid::PIDController, pitch_control::PitchControlMode, Result,
};
use msfs::{
    sim_connect::{data_definition, SimConnect},
    sys::SIMCONNECT_OBJECT_ID_USER,
};

const CONTROL_SURFACES: u32 = 0;

#[data_definition]
#[derive(Default, Clone)]
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

#[derive(Default, Clone)]
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

    pub(crate) fn update(&mut self, ctx: &FBW) -> Result<()> {
        if ctx.data.autopilot() {
            self.surfaces.elevator = ctx.input.yoke_y;
            self.surfaces.ailerons = ctx.input.yoke_x;
            self.surfaces.rudder = ctx.input.rudder;
        } else {
            self.surfaces.elevator = self.pitch_controller.calculate(self.surfaces.elevator, ctx);
            self.surfaces.ailerons = self.roll_controller.calculate(self.surfaces.ailerons, ctx);
            self.surfaces.rudder = ctx.input.rudder; // TODO: yaw FBW
        }

        ctx.sim.set_data_on_sim_object(
            CONTROL_SURFACES,
            SIMCONNECT_OBJECT_ID_USER,
            &self.surfaces,
        )?;

        Ok(())
    }
}

#[derive(Clone)]
struct PitchController {
    aoa_controller: PIDController,
    gforce_controller: PIDController,
}
impl Default for PitchController {
    fn default() -> Self {
        PitchController {
            // AoA error -> elevator handle movement rate
            aoa_controller: PIDController::new(-2.0, 2.0, 0.002, 0.0, 0.0002),
            // GForce error -> elevator handle movement rate
            gforce_controller: PIDController::new(-2.0, 2.0, 0.008, 0.008, 0.001),
        }
    }
}

impl PitchController {
    // Applies load factor limitation protection to a proposed elevator movement
    fn load_factor_limitation(&mut self, delta_elevator: f64, ctx: &FBW) -> f64 {
        let dt = ctx.sim_time.delta();
        if ctx.data.gforce() > ctx.normal_law_protections.max_load_factor {
            self.gforce_controller.update(
                ctx.normal_law_protections.max_load_factor - ctx.data.gforce(),
                dt,
            )
        } else if ctx.data.gforce() < ctx.normal_law_protections.min_load_factor {
            self.gforce_controller.update(
                ctx.normal_law_protections.min_load_factor - ctx.data.gforce(),
                dt,
            )
        } else {
            delta_elevator
        }
    }

    // Applies rules assuming sidestick demands angle of attack
    fn angle_of_attack_demand(&mut self, ctx: &FBW) -> f64 {
        let dt = ctx.sim_time.delta();
        let commanded_aoa = if ctx.input.yoke_y >= 0.0 {
            // Neutral -> Full Up = AoA proportional range from alpha_prot -> alpha_max
            linear_range(
                ctx.input.yoke_y,
                ctx.data.alpha_prot(),
                ctx.data.alpha_max(),
            )
        } else {
            // Neutral -> Full Down = AoA proportional range from alpha_prot -> 0 AoA
            linear_range(ctx.input.yoke_y, ctx.data.alpha_prot(), 0.0)
        };
        let mut delta_elevator = self
            .aoa_controller
            .update(commanded_aoa - ctx.data.alpha(), dt);

        // Apply protections
        delta_elevator = self.load_factor_limitation(delta_elevator, ctx);
        // This isn't specified in the FCOM, but the flight model is not true enough to real life.
        delta_elevator = self.pitch_attitude_protection(delta_elevator);

        delta_elevator
    }

    fn calculate(&mut self, elevator: f64, ctx: &FBW) -> f64 {
        // On the ground, pitch is direct
        // TODO: Add ground mode calculations (e.g. when aircraft reaches 70 knots during the T/O roll, maximum deflection of elevators is affected)
        let new_elevator = if ctx.pitch_control.mode == PitchControlMode::Ground {
            ctx.input.yoke_y
        } else if ctx.normal_law_protections.aoa_demand_active {
            // AoA protections are available in both flight/flare modes
            elevator + self.angle_of_attack_demand(ctx)
        } else if ctx.pitch_control.flare_effect > 0.0 {
            // Flare mode has a special effect and does not have all the protections of flight mode
            42.0
        } else {
            // Flight mode
            43.0
        };

        clamp(new_elevator, -1.0, 1.0)
    }
}

#[derive(Clone)]
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
    fn calculate(&mut self, ailerons: f64, ctx: &FBW) -> f64 {
        // TODO: Handle other control laws besides normal law
        let dt = ctx.sim_time.delta();
        match ctx.pitch_control.mode {
            PitchControlMode::Flight | PitchControlMode::Flare => {
                if ctx.input.yoke_x == 0.0 {
                    // If we are banked beyond the nominal bank angle, roll back to the nominal bank angle
                    if self.roll.abs() > ctx.normal_law_protections.nominal_bank_angle {
                        self.roll += 5.0 * -self.roll.signum() * dt;
                        if self.roll.abs() < ctx.normal_law_protections.nominal_bank_angle {
                            self.roll =
                                self.roll.signum() * ctx.normal_law_protections.nominal_bank_angle;
                        }
                    }
                // We should be holding the specified roll angle
                } else {
                    // We should be responsive to the user's roll request
                    self.roll += 15.0 * ctx.input.yoke_x * dt; // 15 degrees/sec at maximum deflection
                    self.roll = clamp(self.roll, 0.0, 1.0);
                }
                self.controller.update(self.roll - ctx.data.roll(), dt)
            }
            _ => ailerons,
        }
    }
}
