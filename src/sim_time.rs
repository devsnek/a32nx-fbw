use std::time::{SystemTime, UNIX_EPOCH};

pub(crate) struct SimTime {
    previous: SystemTime,
    current: SystemTime,
}

impl Default for SimTime {
    fn default() -> Self {
        let t = SystemTime::now();
        SimTime {
            previous: t,
            current: t,
        }
    }
}

impl SimTime {
    pub(crate) fn init(&mut self) {
        self.update()
    }

    pub(crate) fn update(&mut self) {
        self.previous = self.current;
        self.current = SystemTime::now();
    }

    pub(crate) fn delta(&self) -> f64 {
        self.current
            .duration_since(self.previous)
            .unwrap()
            .as_secs_f64()
    }
    pub(crate) fn current(&self) -> f64 {
        self.current
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64()
    }
}
