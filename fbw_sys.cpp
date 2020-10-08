#include "pitch_control_mode.h"
#include "input.h"
#include "protections.h"
#include "controls.h"
#include "data_recorder.h"

#define ENABLE_DATA_RECORDER TRUE
#define ENABLE_FBW_SYSTEM FALSE

extern "C"
{
	MSFS_CALLBACK bool FBW_gauge_callback(FsContext ctx, const int service_id, void* pData)
	{
		auto ret = true;
		switch (service_id)
		{
		case PANEL_SERVICE_PRE_INSTALL:
		{
			ret &= SUCCEEDED(SimConnect_Open(&hSimConnect, "A32NX_FBW", nullptr, 0, 0, 0));
		}
		break;
		case PANEL_SERVICE_POST_INSTALL:
		{
			sim_time.Update();
			data.Update();
			if (ENABLE_FBW_SYSTEM)
			{
				input_capture.Init();
				control_surfaces.Init();
			}
		}
		break;
		case PANEL_SERVICE_PRE_DRAW:
		{
			// Sent before the gauge is drawn. The pData parameter points to a sGaugeDrawData structure:
			// - The t member gives the absolute simulation time.
			// - The dt member gives the time elapsed since last frame.

			// Example code left for illustration purposes:
			// auto* p_draw_data = static_cast<sGaugeDrawData*>(pData);
			// const auto t = p_draw_data->t;
			// const auto dt = p_draw_data->dt;
		}
		break;
		case PANEL_SERVICE_PRE_UPDATE:
		{
			sim_time.Update();
			data.Update();
			if (ENABLE_DATA_RECORDER)
			{
				data_recorder.Update();
			}
			if (ENABLE_FBW_SYSTEM)
			{
				pitch_control_mode.Update();
				normal_law_protections.Update();
				input_capture.Update();
				control_surfaces.Update(); // Calls the FBW logic internally
			}
		}
		break;
		case PANEL_SERVICE_PRE_KILL:
		{
			if (ENABLE_DATA_RECORDER)
			{
				data_recorder.Destroy();
			}
			ret &= SUCCEEDED(SimConnect_Close(hSimConnect));
		}
		break;
		default: break;
		}
		return ret;
	}
}
