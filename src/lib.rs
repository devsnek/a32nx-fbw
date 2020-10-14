use ::msfs::{
    msfs::{gauge, Gauge, MSFSEvent, PanelServiceID},
    sim_connect::SimConnectRecv,
};

mod controls;
mod data;
mod fbw;
mod input;
mod pid;
mod pitch_control;
mod protections;
mod sim_time;

pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

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

#[gauge(name = FBW)]
async fn fbw_impl(mut gauge: Gauge) -> Result<()> {
    let sim = gauge.open_simconnect("A32NX_FBW")?;
    let mut fbw = fbw::FBW::new(sim);

    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PanelServiceID(service_id) => match service_id {
                PanelServiceID::PostInstall => {
                    fbw.init()?;
                }
                PanelServiceID::PreUpdate => {
                    fbw.update()?;
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
