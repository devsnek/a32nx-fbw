use crate::clamp;

#[derive(Clone)]
pub(crate) struct PIDController {
    output_min: f64,
    output_max: f64,
    kp: f64,
    kd: f64,
    ki: f64,
    integral: f64,
    last_error: f64,
    last_output: f64,
}

impl PIDController {
    pub(crate) fn new(
        output_min: f64,
        output_max: f64,
        kp: f64,
        ki: f64,
        kd: f64,
    ) -> PIDController {
        PIDController {
            output_min,
            output_max,
            kp,
            kd,
            ki,
            integral: 0.0,
            last_error: 0.0,
            last_output: 0.0,
        }
    }

    pub(crate) fn update(&mut self, error: f64, dt: f64) -> f64 {
        // Proportional term
        let p = self.kp * error;

        // Integral term
        self.integral += error * dt;
        let i = self.ki * self.integral;

        // Derivative term
        let d = self.kd * ((error - self.last_error) / dt);

        let output = clamp(p + i + d, self.output_min, self.output_max);

        // Save terms
        self.last_output = output;
        self.last_error = error;

        output
    }

    // Guard against integrator windup
    pub(crate) fn update_anti_windup(&mut self, error: f64, dt: f64) -> f64 {
        if (self.last_output >= self.output_min && self.last_output <= self.output_max)
            || error.signum() != self.last_output.signum()
        {
            self.integral -= error * dt;
        }
        self.update(error, dt)
    }

    pub(crate) fn query(&mut self, error: f64, dt: f64) -> f64 {
        let last_error = self.last_error;
        let last_output = self.last_output;
        let integral = self.integral;
        let update = self.update(error, dt);
        self.last_error = last_error;
        self.last_output = last_output;
        self.integral = integral;
        update
    }
}
