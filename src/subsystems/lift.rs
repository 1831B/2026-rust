//! 2-motor green-cartridge lift subsystem with position holding PID.

use vexide::prelude::*;
use vexide::smart::motor::BrakeMode;
use std::time::Instant;
use crate::config;
use crate::control::pid::Pid;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiftState {
    Idle,
    MovingUp,
    MovingDown,
    Holding,
}

pub struct Lift {
    pub left_motor: Motor,
    pub right_motor: Motor,
    pub state: LiftState,
    pub target_position: f64,
    hold_pid: Pid,
}

impl Lift {
    pub fn new(mut left_motor: Motor, mut right_motor: Motor) -> Self {
        let _ = left_motor.reset_position();
        let _ = left_motor.brake(BrakeMode::Hold);
        let _ = right_motor.reset_position();
        let _ = right_motor.brake(BrakeMode::Hold);

        Self {
            left_motor,
            right_motor,
            state: LiftState::Idle,
            target_position: 0.0,
            hold_pid: Pid::new(3.0, 0.01, 15.0, 40.0, 3.0, 150.0),
        }
    }

    fn set_voltage(&mut self, volts: f64) {
        let volts = volts.clamp(-config::MAX_DRIVE_VOLTAGE, config::MAX_DRIVE_VOLTAGE);
        let _ = self.left_motor.set_voltage(volts);
        let _ = self.right_motor.set_voltage(volts);
    }

    pub fn update(&mut self) {
        if self.state == LiftState::Holding {
            let pos = self.position();
            let error = self.target_position - pos;
            let out_mv = self.hold_pid.update(error).clamp(-12000.0, 12000.0);
            self.set_voltage(out_mv / 1000.0);
        }
    }

    pub fn move_up(&mut self, voltage: f64) {
        self.state = LiftState::MovingUp;
        self.hold_pid.reset();
        self.set_voltage(voltage);
    }

    pub fn move_down(&mut self, voltage: f64) {
        self.state = LiftState::MovingDown;
        self.hold_pid.reset();
        self.set_voltage(-voltage);
    }

    pub fn hold(&mut self) {
        let pos = self.position();
        self.target_position = pos;
        self.state = LiftState::Holding;
        self.hold_pid.reset();
    }

    pub fn stop(&mut self) {
        self.state = LiftState::Idle;
        self.set_voltage(0.0);
        let _ = self.left_motor.brake(BrakeMode::Hold);
        let _ = self.right_motor.brake(BrakeMode::Hold);
    }

    pub async fn move_to_position(&mut self, degrees: f64, timeout_ms: f64) {
        let mut pos_pid = Pid::new(80.0, 0.01, 6.0, 40.0, 3.0, 200.0);
        let start = Instant::now();

        loop {
            let pos = self.position();
            let error = degrees - pos;
            let out_mv = pos_pid.update(error).clamp(-12000.0, 12000.0);
            self.set_voltage(out_mv / 1000.0);

            if pos_pid.is_settled()
                || start.elapsed().as_millis() as f64 > timeout_ms
            {
                break;
            }
            sleep(core::time::Duration::from_millis(10)).await;
        }

        self.target_position = degrees;
        self.state = LiftState::Holding;
        self.hold_pid.reset();
    }

    pub fn position(&self) -> f64 {
        let lp = self.left_motor.position().map(|a| a.as_degrees()).ok();
        let rp = self.right_motor.position().map(|a| a.as_degrees()).ok();
        match (lp, rp) {
            (Some(l), Some(r)) => (l + r) / 2.0,
            (Some(l), None) => l,
            (None, Some(r)) => r,
            (None, None) => 0.0,
        }
    }

    pub fn left_temp(&self) -> f64 {
        self.left_motor.temperature().unwrap_or(0.0)
    }

    pub fn right_temp(&self) -> f64 {
        self.right_motor.temperature().unwrap_or(0.0)
    }

    pub fn is_at_target(&self, tolerance: f64) -> bool {
        (self.target_position - self.position()).abs() < tolerance
    }

    pub fn is_ok(&self) -> (bool, bool) {
        (
            self.left_motor.position().is_ok(),
            self.right_motor.position().is_ok(),
        )
    }
}
