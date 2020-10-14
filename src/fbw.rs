use crate::{
    controls::Controls, data::Data, input::Input, pitch_control::PitchControl,
    protections::NormalLawProtections, sim_time::SimTime, Result,
};

pub(crate) struct FBW {
    pub(crate) sim: msfs::sim_connect::SimConnect,
    pub(crate) sim_time: SimTime,
    pub(crate) input: Input,
    pub(crate) normal_law_protections: NormalLawProtections,
    pub(crate) pitch_control: PitchControl,
    pub(crate) controls: Controls,
    pub(crate) data: Data,
}

impl FBW {
    pub(crate) fn new(sim: msfs::sim_connect::SimConnect) -> Self {
        FBW {
            sim,
            sim_time: Default::default(),
            input: Default::default(),
            pitch_control: Default::default(),
            normal_law_protections: Default::default(),
            controls: Default::default(),
            data: Default::default(),
        }
    }

    pub(crate) fn init(&mut self) -> Result<()> {
        self.sim_time.init();
        self.input.init(&self.sim)?;
        self.controls.init(&self.sim)?;

        Ok(())
    }

    pub(crate) fn update(&mut self) -> Result<()> {
        macro_rules! update {
            ($name:ident) => {
                // These clones are optimized out
                let mut tmp = self.$name.clone();
                tmp.update(self)?;
                self.$name = tmp;
            };
        }

        update!(sim_time);
        update!(data);
        update!(normal_law_protections);
        update!(pitch_control);
        update!(controls);

        Ok(())
    }
}
