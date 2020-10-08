#pragma once
#include "common.h"
#include "data.h"

// Pitch control laws are described in the A320 FCOM 1.27.20
enum class PITCH_CONTROL_MODE
{
	GROUND_MODE,
	FLIGHT_MODE,
	FLARE_MODE
};

class PitchControlMode
{
	PITCH_CONTROL_MODE mode = PITCH_CONTROL_MODE::GROUND_MODE;

	double ground_effect = 1;
	double flight_effect = 0;
	double flare_effect = 0;

	double saved_flare_pitch_attitude = 0; // The pitch attitude of the airplane at 50 feet RA

	void BlendEffect(double * blend_in, double * blend_out, const double increment)
	{
		*blend_in = clamp(*blend_in + increment, 0, 1);
		*blend_out = clamp(*blend_out - increment, 0, 1);
	}

	void HandleGroundTransitions()
	{
		const auto dt = sim_time.DeltaTime();
		// TODO: Handle other laws
		// These transitions are only valid for normal law

		// Handle ground to flight transition
		const auto radio_altimeter = data.RadioHeight();
		const auto in_flight = !data.OnGround();
		const auto pitch_attitude = data.Pitch();
		
		if (radio_altimeter > 50 || (in_flight && pitch_attitude > 8))
		{
			BlendEffect(&flight_effect, &ground_effect, dt / 5);
			if (flight_effect == 1)
			{
				mode = PITCH_CONTROL_MODE::FLIGHT_MODE;
			}
		}
		else
		{
			BlendEffect(&ground_effect, &flight_effect, dt / 5);
		}
	}

	void HandleFlightTransitions()
	{
		const auto dt = sim_time.DeltaTime();
		// TODO: Handle other laws
		// These transitions are only valid for normal law

		// Handle flight to flare transition
		const auto radio_altimeter = data.RadioHeight();
		const auto pitch_attitude = data.Pitch();
		if (radio_altimeter <= 50)
		{
			if (flare_effect == 0)
			{
				// First time blending in the flare effect, so let's save the pitch attitude
				// Why? The FCOM says so:
				// "The system memorizes the attitude at 50 feet, and that attitude becomes the initial reference for pitch attitude control."
				saved_flare_pitch_attitude = pitch_attitude;
			}
			BlendEffect(&flare_effect, &flight_effect, dt / 1);
			if (flare_effect == 1)
			{
				mode = PITCH_CONTROL_MODE::FLARE_MODE;
			}
		}
		else
		{
			BlendEffect(&flight_effect, &flare_effect, dt / 1);
		}
	}

	void HandleFlareTransitions()
	{
		const auto dt = sim_time.DeltaTime();
		// TODO: Handle other laws
		// These transitions are only valid for normal law

		// Handle flare to flight transition
		const auto radio_altimeter = data.RadioHeight();
		const auto on_ground = data.OnGround();
		const auto pitch_attitude = data.Pitch();
		if (radio_altimeter > 50)
		{
			BlendEffect(&flight_effect, &flare_effect, dt / 1);
			if (flight_effect == 1)
			{
				mode = PITCH_CONTROL_MODE::FLIGHT_MODE;
			}
		}
		// Handle flare to ground transition
		else if (on_ground && pitch_attitude < 2.5)
		{
			BlendEffect(&ground_effect, &flare_effect, dt / 5);
			if (ground_effect == 1)
			{
				mode = PITCH_CONTROL_MODE::GROUND_MODE;
			}
		}
		else
		{
			if (ground_effect > 0)
			{
				BlendEffect(&flare_effect, &ground_effect, dt / 5);
			}
			if (flight_effect > 0)
			{
				BlendEffect(&flare_effect, &flight_effect, dt / 1);
			}
		}
		
	}
public:
	PITCH_CONTROL_MODE Mode() { return mode; }
	double GroundEffect() { return ground_effect; }
	double FlightEffect() { return flight_effect; }
	double FlareEffect() { return flare_effect; }
	double SavedFlarePitchAttitude() { return saved_flare_pitch_attitude; }
	
	void Update()
	{
		switch (mode)
		{
		case PITCH_CONTROL_MODE::GROUND_MODE:
			HandleGroundTransitions();
			break;
		case PITCH_CONTROL_MODE::FLIGHT_MODE:
			HandleFlightTransitions();
			break;
		case PITCH_CONTROL_MODE::FLARE_MODE:
			HandleFlareTransitions();
			break;
		}
	}
};
PitchControlMode pitch_control_mode;