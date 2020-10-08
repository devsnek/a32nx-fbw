#pragma once
#include "common.h"

typedef struct DATA_FRAME
{
	double aileron = 0; // Aileron input deflection (-1.0 full left, +1.0 full right)
	double altitude = 0; // The altitude in feet
	double aoa = 0; // The angle of attack in degrees
	bool autopilot = false; // True if the autopilot is on
	double cg_lateral = 0; // The lateral CG as a percent of the reference chord
	double cg_longitudinal = 0; // The longitudinal CG as a percent of the reference chord
	double density = 0; // The density of the air in slugs per cubic feet
	double elevator = 0; // Elevator input deflection (-1.0 full down, +1.0 full up)
	double elevator_trim = 0; // Elevator trim in degrees (+13.5 full up, -4.0 full down)
	double engine_thrust_1 = 0; // Engine #1 thrust in pounds
	double engine_thrust_2 = 0; // Engine #2 thrust in pounds
	int flaps = 0; // The current position of the flaps handle (0 = Clean CONF, 4 = CONF FULL)
	double gforce = 0; // The current gforce (load factor)
	double ias = 0; // The indicated airspeed in knots
	double mach = 0; // The current speed in mach
	double mmo = DBL_MAX; // The Mmo speed in mach
	bool on_ground = true; // True if the plane is on the ground
	double pitch = 0; // Pitch attitude in degrees (+ is up, - is down)
	double radio_height = 0; // Radio altimeter in feet
	double roll = 0; // Roll attitude in degrees (+ is right, - is left)
	double rudder = 0; // Rudder input deflection (-1.0 full down, +1.0 full up)
	bool sim_controllable = false; // True if the sim might be controllable
	double speed_lateral = 0; // Lateral speed (relative to the earth in a north/south direction) in feet/second
	double speed_longitudinal = 0; // Longitudinal speed (relative to the earth in an east/west direction) in feet/second
	double speed_vertical = 0; // Vertical speed (relative to the earth) in feet/second
	double tas = 0; // The true airspeed in knots
	double time = 0; // The simulation time
	double vmo = DBL_MAX; // The Vmo speed in knots
	double weight = 0; // Total weight of the airplane in pounds
	double wind_lateral = 0; // Lateral wind (relative to the earth in a north/south direction) in feet/second
	double wind_longitudinal = 0; // Longitudinal wind (relative to the earth in a east/west direction) in feet/second
	double wind_vertical = 0; // Vertical wind (relative ot the earth) in feet/second
} DATA_FRAME;

const int DATA_FRAME_HISTORY_LENGTH = 50; // 50 samples

class Data
{
	bool initialized = false;
	DATA_FRAME frames[DATA_FRAME_HISTORY_LENGTH];
	
	double FetchSimVar(const char * name, const char * units, const int index, const double fallback)
	{
		const auto value = aircraft_varget(get_aircraft_var_enum(name), get_units_enum(units), index);
		return isnan(value) ? fallback : value;
	}

	DATA_FRAME * PreviousFrame()
	{
		return &frames[DATA_FRAME_HISTORY_LENGTH - 2];
	}

	double VFPAForFrame(DATA_FRAME * frame)
	{
		const auto lateral_speed = (*frame).speed_lateral;
		const auto longitudinal_speed = (*frame).speed_longitudinal;
		const auto vertical_speed = (*frame).speed_vertical;
		const auto horizontal_speed = sqrt(lateral_speed * lateral_speed + longitudinal_speed * longitudinal_speed);
		if (horizontal_speed == 0 && vertical_speed == 0) return 0; // Neutral FPA
		if (horizontal_speed == 0 && vertical_speed < 0) return -90; // Straight down
		if (horizontal_speed == 0 && vertical_speed > 0) return 90; // Straight up
		return degrees(atan(vertical_speed / horizontal_speed));
	}
public:

	double Alpha() { return (*CurrentFrame()).aoa; }
	double AlphaFloor()
	{
		// These values are hardcoded in the FCOM in 1.27.20 under "High Angle of Attack Protection"
		// Note: 2. a.floor is activated through A/THR system when:
		// - a > a floor (9.5 degrees in configuration 0; 15 degrees in configuration 1, 2; 14 degrees in
		//   configuration 3; 13 degrees in configuration FULL), or,...
		// TODO: These values don't seem to mesh well with how the A320 is actually modeled, even though
		//       they come from the manual directly.
		switch ((*CurrentFrame()).flaps)
		{
		case 0: // Clean CONF
			return 9.5;
			break;
		case 1: // CONF 1
		case 2: // CONF 2
			return 15;
			break;
		case 3: // CONF 3
			return 14;
			break;
		case 4: // 
			return 13;
			break;
		default: // Unreachable
			return 9.5;
			break;
		}
	}
	double AlphaProt()
	{
		// This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
		// The graph plots CL (lift coefficient) to alpha.
		// The ratio was guesstimated using a ruler and hoping the graph was accurate.
		const auto ratio_with_alpha_floor = 19.0 / 21.0;
		return ratio_with_alpha_floor * AlphaFloor();
	}
	double AlphaMax()
	{
		// This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
		// The graph plots CL (lift coefficient) to alpha.
		// The ratio was guesstimated using a ruler and hoping the graph was accurate.
		const auto ratio_with_alpha_floor = 7.0 / 6.0;
		return ratio_with_alpha_floor * AlphaFloor();
	}
	bool Autopilot() { return (*CurrentFrame()).autopilot; }
	int Flaps() { return (*CurrentFrame()).flaps; }
	double GForce() { return (*CurrentFrame()).gforce;  }
	double IAS() { return (*CurrentFrame()).ias; }
	double Mach() { return (*CurrentFrame()).mach; }
	double Mmo() { return (*CurrentFrame()).mmo; }
	bool OnGround() { return (*CurrentFrame()).on_ground; }
	double Pitch() { return (*CurrentFrame()).pitch; }
	double PitchRate() { return ((*CurrentFrame()).pitch - (*PreviousFrame()).pitch) / sim_time.DeltaTime(); }
	double RadioHeight() { return (*CurrentFrame()).radio_height; }
	double Roll() { return (*CurrentFrame()).roll;  }
	double VFPA() { return VFPAForFrame(CurrentFrame()); }
	double VFPARate() { return (VFPAForFrame(CurrentFrame()) - VFPAForFrame(PreviousFrame())) / sim_time.DeltaTime(); }
	double Vmo() { return (*CurrentFrame()).vmo; }

	DATA_FRAME * Frames() { return frames; }
	DATA_FRAME * CurrentFrame() { return &frames[DATA_FRAME_HISTORY_LENGTH - 1]; }
	
	void Update()
	{
		// Shift values to the right
		for (auto i = 1; i < DATA_FRAME_HISTORY_LENGTH; i++)
		{
			frames[i - 1] = frames[i];
		}

		// Update the last frame
		auto* frame = &frames[DATA_FRAME_HISTORY_LENGTH - 1];
		
		// Update
		(*frame).aileron = FetchSimVar("AILERON POSITION", "Position", 0, 0);
		(*frame).altitude = FetchSimVar("PLANE ALTITUDE", "Feet", 0, 0);
		(*frame).aoa = FetchSimVar("INCIDENCE ALPHA", "Degrees", 0, 0);
		(*frame).autopilot = FetchSimVar("AUTOPILOT MASTER", "Bool", 0, FALSE) == TRUE;
		(*frame).cg_lateral = FetchSimVar("CG PERCENT LATERAL", "Percent", 0, 0);
		(*frame).cg_longitudinal = FetchSimVar("CG PERCENT", "Percent", 0, 0);
		(*frame).density = FetchSimVar("AMBIENT DENSITY", "Slugs per cubic feet", 0, 0);
		(*frame).elevator = FetchSimVar("ELEVATOR POSITION", "Position", 0, 0);
		(*frame).elevator_trim = FetchSimVar("ELEVATOR TRIM POSITION", "Degrees", 0, 0);
		(*frame).engine_thrust_1 = FetchSimVar("TURB ENG JET THRUST", "Pounds", 1, 0);
		(*frame).engine_thrust_2 = FetchSimVar("TURB ENG JET THRUST", "Pounds", 2, 0);
		(*frame).flaps = static_cast<int>(FetchSimVar("FLAPS HANDLE INDEX", "Number", 0, 0));
		(*frame).gforce = FetchSimVar("G FORCE", "GForce", 0, 0);
		(*frame).ias = FetchSimVar("AIRSPEED INDICATED", "Knots", 0, 0);
		(*frame).mach = FetchSimVar("AIRSPEED MACH", "Mach", 0, 0);
		(*frame).mmo = FetchSimVar("BARBER POLE MACH", "Mach", 0, DBL_MAX); // TODO: Get this data from the FCOM instead of the SimVar
		(*frame).on_ground = FetchSimVar("SIM ON GROUND", "Bool", 0, FALSE) == TRUE;
		(*frame).pitch = -FetchSimVar("PLANE PITCH DEGREES", "Degrees", 0, 0);
		(*frame).radio_height = FetchSimVar("RADIO HEIGHT", "Feet", 0, 0);
		(*frame).roll = -FetchSimVar("PLANE BANK DEGREES", "Degrees", 0, 0);
		(*frame).rudder = FetchSimVar("RUDDER POSITION", "Position", 0, 0);
		(*frame).sim_controllable = FetchSimVar("IS LATITUDE LONGITUDE FREEZE ON", "Bool", 0, FALSE) == FALSE
			&& FetchSimVar("IS ALTITUDE FREEZE ON", "Bool", 0, FALSE) == FALSE
			&& FetchSimVar("IS ATTITUDE FREEZE ON", "Bool", 0, FALSE) == FALSE
			&& FetchSimVar("SIM DISABLED", "Bool", 0, FALSE) == FALSE
			&& FetchSimVar("IS SLEW ACTIVE", "Bool", 0, FALSE) == FALSE;
		(*frame).speed_lateral = FetchSimVar("VELOCITY WORLD Z", "Feet per second", 0, 0);
		(*frame).speed_longitudinal = FetchSimVar("VELOCITY WORLD X", "Feet per second", 0, 0);
		(*frame).speed_vertical = FetchSimVar("VELOCITY WORLD Y", "Feet per second", 0, 0);
		(*frame).tas = FetchSimVar("AIRSPEED TRUE", "Knots", 0, 0);
		(*frame).time = sim_time.CurrentTime();
		(*frame).vmo = FetchSimVar("AIRSPEED BARBER POLE", "Knots", 0, DBL_MAX); // TODO: Get this data from the FCOM instead of the SimVar
		(*frame).weight = FetchSimVar("TOTAL WEIGHT", "Pounds", 0, 0);
		(*frame).wind_lateral = FetchSimVar("AMBIENT WIND Z", "Feet per second", 0, 0);
		(*frame).wind_longitudinal = FetchSimVar("AMBIENT WIND X", "Feet per second", 0, 0);
		(*frame).wind_vertical = FetchSimVar("AMBIENT WIND Y", "Feet per second", 0, 0);
		
		// If we've only ever seen a single data point, copy that to all data points
		if (!initialized)
		{
			for (auto i = 0; i < DATA_FRAME_HISTORY_LENGTH - 1; i++)
			{
				frames[i] = frames[DATA_FRAME_HISTORY_LENGTH - 1];
			}
			initialized = true;
		}
	}
};

Data data;