use crate::Result;
use msfs::{
    sim_connect::SimConnect,
    sys::{SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_RECV_EVENT},
};

#[repr(u32)]
enum EventID {
    // Elevator Group
    ElevatorSet, // AXIS_ELEVATOR_SET
    // Aileron Group
    AileronsSet,          // AXIS_AILERONS_SET
    CenterAileronsRudder, // CENTER_AILER_RUDDER
    // Rudder group
    RudderSet,    // AXIS_RUDDER_SET
    RudderCenter, // RUDDER_CENTER
}
impl From<u32> for EventID {
    fn from(v: u32) -> Self {
        unsafe { std::mem::transmute(v) }
    }
}

#[repr(u32)]
enum GroupID {
    Elevator,
    Ailerons,
    Rudder,
}

#[derive(Default)]
pub(crate) struct Input {
    pub(crate) yoke_y: f64, // -1 is full down, and +1 is full up
    pub(crate) yoke_x: f64, // -1 is full left, and +1 is full right
    pub(crate) rudder: f64, // -1 is full left, and +1 is full right
}

impl Input {
    pub(crate) fn init(&self, sim: &SimConnect) -> Result<()> {
        // Elevator group
        sim.map_client_event_to_sim_event(EventID::ElevatorSet as u32, "AXIS_ELEVATOR_SET")?;
        sim.add_client_event_to_notification_group(
            GroupID::Elevator as u32,
            EventID::ElevatorSet as u32,
            true,
        )?;

        // Ailerons group
        sim.map_client_event_to_sim_event(EventID::AileronsSet as u32, "AXIS_AILERONS_SET")?;
        sim.map_client_event_to_sim_event(
            EventID::CenterAileronsRudder as u32,
            "CENTER_AILER_RUDDER",
        )?;
        sim.add_client_event_to_notification_group(
            GroupID::Ailerons as u32,
            EventID::AileronsSet as u32,
            true,
        )?;
        sim.add_client_event_to_notification_group(
            GroupID::Ailerons as u32,
            EventID::CenterAileronsRudder as u32,
            true,
        )?;

        // Rudder group
        sim.map_client_event_to_sim_event(EventID::RudderSet as u32, "AXIS_RUDDER_SET")?;
        sim.map_client_event_to_sim_event(EventID::RudderCenter as u32, "RUDDER_CENTER")?;
        sim.add_client_event_to_notification_group(
            GroupID::Rudder as u32,
            EventID::RudderSet as u32,
            true,
        )?;
        sim.add_client_event_to_notification_group(
            GroupID::Rudder as u32,
            EventID::RudderCenter as u32,
            true,
        )?;

        // Set maskable notification priorities
        sim.set_notification_group_priority(
            GroupID::Elevator as u32,
            SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE,
        )?;
        sim.set_notification_group_priority(
            GroupID::Ailerons as u32,
            SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE,
        )?;
        sim.set_notification_group_priority(
            GroupID::Rudder as u32,
            SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE,
        )?;

        Ok(())
    }

    pub(crate) fn update(&mut self, event: &SIMCONNECT_RECV_EVENT) -> Result<()> {
        // scale from [-16384, 16384] to [-1,1] and reverse the sign
        let map = |n| 0.0 - (n as f64 / 16384.0);

        match event.uEventID.into() {
            EventID::ElevatorSet => {
                self.yoke_y = map(event.dwData);
            }
            EventID::AileronsSet => {
                self.yoke_x = map(event.dwData);
            }
            EventID::CenterAileronsRudder => {
                self.yoke_x = 0.0;
                self.rudder = 0.0;
            }
            EventID::RudderSet => {
                self.rudder = map(event.dwData);
            }
            EventID::RudderCenter => {
                self.rudder = 0.0;
            }
        }

        Ok(())
    }
}
