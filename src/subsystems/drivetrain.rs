use vexide::prelude::*;
use vexide::smart::motor::BrakeMode;
use crate::config;

pub struct Drivetrain {
    pub left_motors: [Motor; 3],
    pub right_motors: [Motor; 3],
}

impl Drivetrain {
    pub fn new(
        left_front: Motor,
        left_mid: Motor,
        left_back: Motor,
        right_front: Motor,
        right_mid: Motor,
        right_back: Motor,
    ) -> Self {
        let mut dt = Self {
            left_motors: [left_front, left_mid, left_back],
            right_motors: [right_front, right_mid, right_back],
        };
        dt.coast();
        dt
    }

    pub fn drive_voltage(&mut self, left_v: f64, right_v: f64) {
        let left_v = left_v.clamp(-config::MAX_DRIVE_VOLTAGE, config::MAX_DRIVE_VOLTAGE);
        let right_v = right_v.clamp(-config::MAX_DRIVE_VOLTAGE, config::MAX_DRIVE_VOLTAGE);

        for m in &mut self.left_motors {
            let _ = m.set_voltage(left_v);
        }
        for m in &mut self.right_motors {
            let _ = m.set_voltage(right_v);
        }
    }

    pub fn drive_velocity(&mut self, left_pct: f64, right_pct: f64) {
        let left_pct = left_pct.clamp(-100.0, 100.0);
        let right_pct = right_pct.clamp(-100.0, 100.0);
        let left_rpm = (left_pct * 6.0) as i32;
        let right_rpm = (right_pct * 6.0) as i32;

        for m in &mut self.left_motors {
            let _ = m.set_velocity(left_rpm);
        }
        for m in &mut self.right_motors {
            let _ = m.set_velocity(right_rpm);
        }
    }

    pub fn brake(&mut self) {
        for m in &mut self.left_motors {
            let _ = m.brake(BrakeMode::Brake);
            let _ = m.set_voltage(0.0);
        }
        for m in &mut self.right_motors {
            let _ = m.brake(BrakeMode::Brake);
            let _ = m.set_voltage(0.0);
        }
    }

    pub fn coast(&mut self) {
        for m in &mut self.left_motors {
            let _ = m.brake(BrakeMode::Coast);
            let _ = m.set_voltage(0.0);
        }
        for m in &mut self.right_motors {
            let _ = m.brake(BrakeMode::Coast);
            let _ = m.set_voltage(0.0);
        }
    }

    pub fn emergency_stop(&mut self) {
        for m in &mut self.left_motors {
            let _ = m.set_voltage(0.0);
            let _ = m.brake(BrakeMode::Hold);
        }
        for m in &mut self.right_motors {
            let _ = m.set_voltage(0.0);
            let _ = m.brake(BrakeMode::Hold);
        }
    }

    pub fn left_position(&self) -> f64 {
        let mut count = 0;
        let mut sum = 0.0;
        for m in &self.left_motors {
            if let Ok(pos) = m.position() {
                sum += pos.as_degrees();
                count += 1;
            }
        }
        if count > 0 { sum / count as f64 } else { 0.0 }
    }

    pub fn right_position(&self) -> f64 {
        let mut count = 0;
        let mut sum = 0.0;
        for m in &self.right_motors {
            if let Ok(pos) = m.position() {
                sum += pos.as_degrees();
                count += 1;
            }
        }
        if count > 0 { sum / count as f64 } else { 0.0 }
    }

    pub fn left_velocity(&self) -> f64 {
        let mut count = 0;
        let mut sum = 0.0;
        for m in &self.left_motors {
            if let Ok(vel) = m.velocity() {
                sum += vel;
                count += 1;
            }
        }
        if count > 0 { sum / count as f64 } else { 0.0 }
    }

    pub fn right_velocity(&self) -> f64 {
        let mut count = 0;
        let mut sum = 0.0;
        for m in &self.right_motors {
            if let Ok(vel) = m.velocity() {
                sum += vel;
                count += 1;
            }
        }
        if count > 0 { sum / count as f64 } else { 0.0 }
    }

    pub fn left_current(&self) -> f64 {
        self.left_motors.iter().filter_map(|m| m.current().ok()).sum()
    }

    pub fn right_current(&self) -> f64 {
        self.right_motors.iter().filter_map(|m| m.current().ok()).sum()
    }

    pub fn left_temp(&self) -> f64 {
        self.left_motors
            .iter()
            .filter_map(|m| m.temperature().ok())
            .fold(0.0, f64::max)
    }

    pub fn right_temp(&self) -> f64 {
        self.right_motors
            .iter()
            .filter_map(|m| m.temperature().ok())
            .fold(0.0, f64::max)
    }

    pub fn is_ok(&self) -> (bool, bool) {
        let left_ok = self.left_motors.iter().any(|m| m.position().is_ok());
        let right_ok = self.right_motors.iter().any(|m| m.position().is_ok());
        (left_ok, right_ok)
    }
}
