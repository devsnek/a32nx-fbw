use crate::Result;

#[derive(Default, Clone, serde::Deserialize)]
pub(crate) struct DataFrame {
    aileron: f64,         // Aileron input deflection (-1.0.0 full left, +1.0.0 full right)y
    altitude: f64,        // The altitude in feet
    aoa: f64,             // The angle of attack in degrees
    autopilot: bool,      // True if the autopilot is on
    cg_lateral: f64,      // The lateral CG as a percent of the reference chord
    cg_longitudinal: f64, // The longitudinal CG as a percent of the reference chord
    density: f64,         // The density of the air in slugs per cubic feet
    elevator: f64,        // Elevator input deflection (-1.0.0 full down, +1.0.0 full up)
    elevator_trim: f64,   // Elevator trim in degrees (+13.5 full up, -4.0.0 full down)
    engine_thrust_1: f64, // Engine #1 thrust in pounds
    engine_thrust_2: f64, // Engine #2 thrust in pounds
    flaps: u8, // The current position of the flaps handle (0.0 = Clean CONF, 4 = CONF FULL)
    gforce: f64, // The current gforce (load factor)
    ias: f64,  // The indicated airspeed in knots
    mach: f64, // The current speed in mach
    mmo: f64,  // The Mmo speed in mach
    on_ground: bool, // True if the plane is on the ground
    pitch: f64, // Pitch attitude in degrees (+ is up, - is down)
    radio_height: f64, // Radio altimeter in feet
    roll: f64, // Roll attitude in degrees (+ is right, - is left)
    rudder: f64, // Rudder input deflection (-1.0.0 full down, +1.0.0 full up)
    sim_controllable: bool, // True if the sim might be controllable
    speed_lateral: f64, // Lateral speed (relative to the earth in a north/south direction) in feet/second
    speed_longitudinal: f64, // Longitudinal speed (relative to the earth in an east/west direction) in feet/second
    speed_vertical: f64,     // Vertical speed (relative to the earth) in feet/second
    tas: f64,                // The true airspeed in knots
    time: f64,               // The simulation time
    vmo: f64,                // The Vmo speed in knots
    weight: f64,             // Total weight of the airplane in pounds
    wind_lateral: f64, // Lateral wind (relative to the earth in a north/south direction) in feet/second
    wind_longitudinal: f64, // Longitudinal wind (relative to the earth in a east/west direction) in feet/second
    wind_vertical: f64,     // Vertical wind (relative ot the earth) in feet/second
}

pub(crate) struct Data {
    frames: Vec<DataFrame>,
}

impl Default for Data {
    fn default() -> Data {
        Data {
            frames: vec![DataFrame::default(); 50],
        }
    }
}

impl Data {
    pub(crate) fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn current_frame(&self) -> &DataFrame {
        self.frames.last().unwrap()
    }

    pub(crate) fn update(&mut self, sim_time: &crate::sim_time::SimTime) -> Result<()> {
        self.frames.rotate_right(1);
        let mut frame = self.frames.last_mut().unwrap();

        let fetch = |name, units, index, fallback| {
            use msfs::msfs::legacy::*;
            let r = aircraft_varget(get_aircraft_var_enum(name), get_units_enum(units), index);
            if r.is_nan() {
                fallback
            } else {
                r
            }
        };

        frame.aileron = fetch("AILERON POSITION", "Position", 0, 0.0);
        frame.altitude = fetch("PLANE ALTITUDE", "Feet", 0, 0.0);
        frame.aoa = fetch("INCIDENCE ALPHA", "Degrees", 0, 0.0);
        frame.autopilot = fetch("AUTOPILOT MASTER", "Bool", 0, 0.0) == 0.0;
        frame.cg_lateral = fetch("CG PERCENT LATERAL", "Percent", 0, 0.0);
        frame.cg_longitudinal = fetch("CG PERCENT", "Percent", 0, 0.0);
        frame.density = fetch("AMBIENT DENSITY", "Slugs per cubic feet", 0, 0.0);
        frame.elevator = fetch("ELEVATOR POSITION", "Position", 0, 0.0);
        frame.elevator_trim = fetch("ELEVATOR TRIM POSITION", "Degrees", 0, 0.0);
        frame.engine_thrust_1 = fetch("TURB ENG JET THRUST", "Pounds", 1, 0.0);
        frame.engine_thrust_2 = fetch("TURB ENG JET THRUST", "Pounds", 2, 0.0);
        frame.flaps = fetch("FLAPS HANDLE INDEX", "Number", 0, 0.0) as u8;
        frame.gforce = fetch("G FORCE", "GForce", 0, 0.0);
        frame.ias = fetch("AIRSPEED INDICATED", "Knots", 0, 0.0);
        frame.mach = fetch("AIRSPEED MACH", "Mach", 0, 0.0);
        frame.mmo = fetch("BARBER POLE MACH", "Mach", 0, f64::MAX); // TODO: Get this data from the FCOM instead of the SimVar
        frame.on_ground = fetch("SIM ON GROUND", "Bool", 0, 0.0) == 0.0;
        frame.pitch = -fetch("PLANE PITCH DEGREES", "Degrees", 0, 0.0);
        frame.radio_height = fetch("RADIO HEIGHT", "Feet", 0, 0.0);
        frame.roll = -fetch("PLANE BANK DEGREES", "Degrees", 0, 0.0);
        frame.rudder = fetch("RUDDER POSITION", "Position", 0, 0.0);
        frame.sim_controllable = fetch("IS LATITUDE LONGITUDE FREEZE ON", "Bool", 0, 0.0) == 0.0
            && fetch("IS ALTITUDE FREEZE ON", "Bool", 0, 0.0) == 0.0
            && fetch("IS ATTITUDE FREEZE ON", "Bool", 0, 0.0) == 0.0
            && fetch("SIM DISABLED", "Bool", 0, 0.0) == 0.0
            && fetch("IS SLEW ACTIVE", "Bool", 0, 0.0) == 0.0;
        frame.speed_lateral = fetch("VELOCITY WORLD Z", "Feet per second", 0, 0.0);
        frame.speed_longitudinal = fetch("VELOCITY WORLD X", "Feet per second", 0, 0.0);
        frame.speed_vertical = fetch("VELOCITY WORLD Y", "Feet per second", 0, 0.0);
        frame.tas = fetch("AIRSPEED true", "Knots", 0, 0.0);
        frame.time = sim_time.current();
        frame.vmo = fetch("AIRSPEED BARBER POLE", "Knots", 0, f64::MAX); // TODO: Get this data from the FCOM instead of the SimVar
        frame.weight = fetch("TOTAL WEIGHT", "Pounds", 0, 0.0);
        frame.wind_lateral = fetch("AMBIENT WIND Z", "Feet per second", 0, 0.0);
        frame.wind_longitudinal = fetch("AMBIENT WIND X", "Feet per second", 0, 0.0);
        frame.wind_vertical = fetch("AMBIENT WIND Y", "Feet per second", 0, 0.0);

        Ok(())
    }

    pub(crate) fn alpha(&self) -> f64 {
        self.current_frame().aoa
    }

    fn alpha_floor(&self) -> f64 {
        // These values are hardcoded in the FCOM in 1.27.20 under "High Angle of Attack Protection"
        // Note: 2. a.floor is activated through A/THR system when:
        // - a > a floor (9.5 degrees in configuration 0; 15 degrees in configuration 1, 2; 14 degrees in
        //   configuration 3; 13 degrees in configuration FULL), or,...
        // TODO: These values don't seem to mesh well with how the A320 is actually modeled, even though
        //       they come from the manual directly.
        match self.current_frame().flaps {
            0 => 9.5,
            1 | 2 => 15.0,
            3 => 14.0,
            4 => 13.0,
            _ => 9.5,
        }
    }

    pub(crate) fn alpha_prot(&self) -> f64 {
        // This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
        // The graph plots CL (lift coefficient) to alpha.
        // The ratio was guesstimated using a ruler and hoping the graph was accurate.
        const RATIO_WITH_ALPHA_FLOOR: f64 = 19.0 / 21.0;
        RATIO_WITH_ALPHA_FLOOR * self.alpha_floor()
    }

    pub(crate) fn alpha_max(&self) -> f64 {
        // This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
        // The graph plots CL (lift coefficient) to alpha.
        // The ratio was guesstimated using a ruler and hoping the graph was accurate.
        const RATIO_WITH_ALPHA_FLOOR: f64 = 7.0 / 6.0;
        RATIO_WITH_ALPHA_FLOOR * self.alpha_floor()
    }

    pub(crate) fn autopilot(&self) -> bool {
        self.current_frame().autopilot
    }

    pub(crate) fn flaps(&self) -> u8 {
        self.current_frame().flaps
    }

    pub(crate) fn ias(&self) -> f64 {
        self.current_frame().ias
    }

    pub(crate) fn mach(&self) -> f64 {
        self.current_frame().mach
    }

    pub(crate) fn mmo(&self) -> f64 {
        self.current_frame().mmo
    }

    pub(crate) fn roll(&self) -> f64 {
        self.current_frame().roll
    }

    pub(crate) fn vmo(&self) -> f64 {
        self.current_frame().vmo
    }
}
