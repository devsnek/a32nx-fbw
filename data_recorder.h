#pragma once
#include "common.h"
#include "data.h"

class DataRecorder
{
	FILE* file = nullptr;
	long num_valid_updates_seen = 0;

	void WriteData(DATA_FRAME * frame)
	{
		fprintf(file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			(*frame).aileron,
			(*frame).altitude,
			(*frame).aoa,
			(*frame).cg_lateral,
			(*frame).cg_longitudinal,
			(*frame).density,
			(*frame).elevator,
			(*frame).elevator_trim,
			(*frame).engine_thrust_1,
			(*frame).engine_thrust_2,
			(*frame).flaps,
			(*frame).gforce,
			(*frame).ias,
			(*frame).mach,
			(*frame).pitch,
			(*frame).radio_height,
			(*frame).roll,
			(*frame).rudder,
			(*frame).speed_lateral,
			(*frame).speed_longitudinal,
			(*frame).speed_vertical,
			(*frame).tas,
			(*frame).time,
			(*frame).weight,
			(*frame).wind_lateral,
			(*frame).wind_longitudinal,
			(*frame).wind_vertical);
	}

public:
	void Update()
	{
		const auto autopilot_active = (*data.CurrentFrame()).autopilot;
		const auto sim_controllable = (*data.CurrentFrame()).sim_controllable;
		const auto sim_on_ground = (*data.CurrentFrame()).on_ground;

		// We don't want to capture bad data: when the autopilot is running, the sim isn't running, or the sim is on the round
		if (autopilot_active || !sim_controllable || sim_on_ground)
		{
			num_valid_updates_seen = 0;
			Destroy(); // We close the current file if we have it open
			return;
		}

		// We do see valid data
		num_valid_updates_seen++;

		if (num_valid_updates_seen < DATA_FRAME_HISTORY_LENGTH) return; // We need to collect more data

		// We have just enough data to write it to a file
		if (num_valid_updates_seen == DATA_FRAME_HISTORY_LENGTH)
		{
			// Open a new file and write the first (n - 1) data points
			const auto filename = std::string(R"(SimObjects\AirPlanes\Asobo_A320_NEO\FlightDataRecorder-)")
				+ std::to_string(static_cast<long long>(sim_time.CurrentTime()))
				+ std::string(".csv");
			printf("Recording flight data to file to file %s\n", filename.c_str());
			file = fopen(filename.c_str(), "w");
			for (auto i = 0; i < DATA_FRAME_HISTORY_LENGTH - 1; i++)
			{
				WriteData(&data.Frames()[i]);
			}
		}

		// Write the last (nth) data point to the file
		if (num_valid_updates_seen >= DATA_FRAME_HISTORY_LENGTH)
		{
			WriteData(&data.Frames()[DATA_FRAME_HISTORY_LENGTH - 1]);
		}
	}

	void Destroy()
	{
		if (file != nullptr)
		{
			fclose(file);
			file = nullptr;
		}
	}
};

DataRecorder data_recorder;