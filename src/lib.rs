use ::msfs::{
    msfs::{gauge, Gauge, MSFSEvent, PanelServiceID},
    sim_connect::{SimConnect, SimConnectRecv},
};

mod controls;
mod data;
mod input;
mod pid;
mod pitch_control;
mod protections;
mod sim_time;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

// TODO: replace with f64::clamp when stable
pub(crate) fn clamp(mut x: f64, min: f64, max: f64) -> f64 {
    debug_assert!(min < max);
    if x < min {
        x = min;
    }
    if x > max {
        x = max;
    }
    x
}

pub(crate) fn linear_range(coefficient: f64, min: f64, max: f64) -> f64 {
    (max - min) * coefficient + min
}

#[derive(Default)]
struct FBW {
    sim_time: sim_time::SimTime,
    input: input::Input,
    pitch_control: pitch_control::PitchControl,
    normal_law_protections: protections::NormalLawProtections,
    controls: controls::Controls,
    data: data::Data,
}

impl FBW {
    fn init(&mut self, sim: &SimConnect) -> Result<()> {
        self.sim_time.init();
        self.data.init()?;
        self.input.init(&sim)?;
        self.controls.init(&sim)?;

        Ok(())
    }

    fn update(&mut self, sim: &SimConnect) -> Result<()> {
        self.sim_time.update();
        self.data.update(&self.sim_time)?;

        self.normal_law_protections
            .update(&self.sim_time, &self.data, &self.input);
        self.pitch_control.update()?;
        self.controls.update(
            sim,
            &self.sim_time,
            &self.data,
            &self.normal_law_protections,
            &self.pitch_control,
            &self.input,
        )?;

        Ok(())
    }
}

#[gauge(name = FBW)]
async fn fbw(mut gauge: Gauge) -> Result<()> {
    let sim = gauge.open_simconnect("A32NX_FBW")?;

    let mut fbw = FBW::default();

    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PanelServiceID(service_id) => match service_id {
                PanelServiceID::PostInstall => {
                    fbw.init(&sim)?;
                }
                PanelServiceID::PreUpdate => {
                    fbw.update(&sim)?;
                }
                _ => {}
            },
            MSFSEvent::SimConnect(recv) => {
                if let SimConnectRecv::Event(event) = recv {
                    fbw.input.update(event)?;
                }
            }
        }
    }

    Ok(())
}
