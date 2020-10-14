use crate::{fbw::FBW, Result};

#[derive(Default, Clone)]
pub(crate) struct NormalLawProtections {
    pub(crate) aoa_demand_active: bool,
    pub(crate) aoa_demand_deactivation_timer: f64,
    pub(crate) high_speed_protection_active: bool,
    pub(crate) max_bank_angle: f64,
    pub(crate) nominal_bank_angle: f64,
    pub(crate) min_load_factor: f64,
    pub(crate) max_load_factor: f64,
    pub(crate) min_pitch_angle: f64,
    pub(crate) max_pitch_angle: f64,
}

impl NormalLawProtections {
    // Maximum bank angle in degrees
    // - Normally: 67 degrees
    // - High Angle of Attack Protection: 45 degrees
    // - High Speed Protection: 45 degrees
    const MAX_BANK_ANGLE_NORMAL: f64 = 67.0;
    const MAX_BANK_ANGLE_PROTECTED: f64 = 45.0;

    // Spiral static stability in degrees
    // - Normally: 33 degrees
    // - High Angle of Attack Protection: 0 degrees
    // - High Speed Protection: 0 degrees
    const NOMINAL_BANK_ANGLE_NORMAL: f64 = 33.0;
    const NOMINAL_BANK_ANGLE_PROTECTED: f64 = 0.0;

    // -1g for clean configuration, 0g for other configurations
    const MIN_LOAD_FACTOR_NORMAL: f64 = -1.0;
    const MIN_LOAD_FACTOR_PROTECTED: f64 = 0.0;
    // 2.5g for clean configuration, 2g for other configurations
    const MAX_LOAD_FACTOR_NORMAL: f64 = 2.5;
    const MAX_LOAD_FACTOR_PROTECTED: f64 = 2.0;

    // Maximum pitch attitude in degrees
    // 30 degrees nose up in conf 0-3 (progressively reduced to 25 degrees at low speed)
    // 25 degrees nose up in conf FULL (progressively reduced to 20 degrees at low speed)
    // TODO: To implement the 'progressively reduced' limitation, we need to find a way to
    //       calculate speeds like V_alpha_prot or V_alpha_max, for which I think we will
    //       have to work out via experimentation.
    const MAX_PITCH_ANGLE_NORMAL: f64 = 30.0;
    const MAX_PITCH_ANGLE_PROTECTED: f64 = 25.0;

    // Minimum pitch attitude in degrees
    const MIN_PITCH_ANGLE_NORMAL: f64 = -15.0;
}

impl NormalLawProtections {
    pub(crate) fn update(&mut self, ctx: &FBW) -> Result<()> {
        let dt = ctx.sim_time.delta();

        // Check if we are in AoA demand mode (as dictated by the High Angle of Attack Protection)
        if self.aoa_demand_active {
            // Should we leave AoA demand mode?
            // Exit condition 1: Sidestick must be pushed more than 8 degrees forward (assuming self is ~50% down)
            let condition1 = ctx.input.yoke_y < -0.5;
            // Exit condition 2: Sidestick must be pushed more than 0.5 degrees forward for at least 0.5 seconds when alpha < alpha_max
            let condition2 = self.aoa_demand_deactivation_timer >= 0.5;
            if condition1 || condition2 {
                self.aoa_demand_active = false;
                self.aoa_demand_deactivation_timer = 0.0;
            } else if ctx.input.yoke_y < 0.0 && ctx.data.alpha() < ctx.data.alpha_max() {
                // We're still building the target duration to meet condition 2
                self.aoa_demand_deactivation_timer += dt;
            } else {
                // We don't match any of the exit conditions
                self.aoa_demand_deactivation_timer = 0.0;
            }
        } else {
            // Should we enter AoA demand mode?
            // Enter condition 1: Sidestick must not be pushed down, and AoA is greater than alpha_prot
            let condition1 = ctx.input.yoke_y >= 0.0 && ctx.data.alpha() > ctx.data.alpha_prot();
            // Enter condition 2: We are at or above alpha max
            let condition2 = ctx.data.alpha() >= ctx.data.alpha_max();
            if condition1 || condition2 {
                self.aoa_demand_active = true;
                self.aoa_demand_deactivation_timer = 0.0;
            }
        }

        // Check if high speed protection is active
        self.high_speed_protection_active =
            ctx.data.ias() > ctx.data.vmo() || ctx.data.mach() > ctx.data.mmo();

        // Update bank angle limits
        if self.aoa_demand_active || self.high_speed_protection_active {
            self.max_bank_angle = Self::MAX_BANK_ANGLE_PROTECTED;
            self.nominal_bank_angle = Self::NOMINAL_BANK_ANGLE_PROTECTED;
        } else {
            self.max_bank_angle = Self::MAX_BANK_ANGLE_NORMAL;
            self.nominal_bank_angle = Self::NOMINAL_BANK_ANGLE_NORMAL;
        }

        // Update load and pitch factors
        self.min_pitch_angle = Self::MIN_PITCH_ANGLE_NORMAL;
        match ctx.data.flaps() {
            0 => {
                self.min_load_factor = Self::MIN_LOAD_FACTOR_NORMAL;
                self.max_load_factor = Self::MAX_LOAD_FACTOR_NORMAL;
                self.max_pitch_angle = Self::MAX_PITCH_ANGLE_NORMAL;
            }
            1 | 2 | 3 => {
                self.min_load_factor = Self::MIN_LOAD_FACTOR_PROTECTED;
                self.max_load_factor = Self::MAX_LOAD_FACTOR_PROTECTED;
                self.max_pitch_angle = Self::MAX_PITCH_ANGLE_NORMAL;
            }
            4 => {
                self.min_load_factor = Self::MIN_LOAD_FACTOR_PROTECTED;
                self.max_load_factor = Self::MAX_LOAD_FACTOR_PROTECTED;
                self.max_pitch_angle = Self::MAX_PITCH_ANGLE_PROTECTED;
            }
            _ => unreachable!(),
        };

        Ok(())
    }
}
