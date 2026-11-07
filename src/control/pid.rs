//! Proportional-Integral-Derivative (PID) controller with anti-windup and settle detection.

use std::time::Instant;

#[derive(Debug, Clone)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub settle_error: f64,
    pub settle_time_ms: f64,

    integral: f64,
    prev_error: f64,
    settle_timer_ms: f64,
    prev_time: Option<Instant>,
    first_run: bool,
}

impl Pid {
    pub fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        integral_limit: f64,
        settle_error: f64,
        settle_time_ms: f64,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            settle_error,
            settle_time_ms,
            integral: 0.0,
            prev_error: 0.0,
            settle_timer_ms: 0.0,
            prev_time: None,
            first_run: true,
        }
    }

    pub fn update(&mut self, error: f64) -> f64 {
        let now = Instant::now();

        if self.first_run {
            self.prev_error = error;
            self.prev_time = Some(now);
            self.first_run = false;
            return self.kp * error;
        }

        let dt = match self.prev_time {
            Some(prev) => {
                let duration = now.duration_since(prev);
                let secs = duration.as_secs_f64();
                if secs <= 0.0 { 0.01 } else { secs }
            }
            None => 0.01,
        };

        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);

        // Reset integral when error crosses zero or within small threshold
        if error.abs() < self.settle_error * 0.1 {
            self.integral = 0.0;
        }

        let derivative = (error - self.prev_error) / dt;

        if error.abs() < self.settle_error {
            self.settle_timer_ms += dt * 1000.0;
        } else {
            self.settle_timer_ms = 0.0;
        }

        self.prev_error = error;
        self.prev_time = Some(now);

        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.settle_timer_ms = 0.0;
        self.prev_time = None;
        self.first_run = true;
    }

    pub fn is_settled(&self) -> bool {
        self.settle_timer_ms >= self.settle_time_ms
    }

    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
}
