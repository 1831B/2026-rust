use vexide::prelude::*;
use vexide::math::Angle;
use vexide::smart::motor::BrakeMode;
use std::time::Instant;
use crate::config;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClawState {
    Open,
    Closed,
    Moving,
}

pub struct TopMechanism {
    pub claw_motor: Motor,
    pub claw_state: ClawState,
}

impl TopMechanism {
    pub fn new(mut claw_motor: Motor) -> Self {
        let _ = claw_motor.reset_position();
        let _ = claw_motor.brake(BrakeMode::Hold);

        Self {
            claw_motor,
            claw_state: ClawState::Open,
        }
    }

    fn motor_to_position(m: &mut Motor, deg: f64, speed_pct: f64) {
        let rpm = (speed_pct * 2.0) as i32;
        let _ = m.set_position_target(Angle::from_degrees(deg), rpm);
    }

    pub fn update(&mut self) {}

    pub fn claw_open(&mut self) {
        self.claw_state = ClawState::Moving;
        Self::motor_to_position(&mut self.claw_motor, config::CLAW_OPEN_POS, config::CLAW_SPEED_PCT);
        self.claw_state = ClawState::Open;
    }

    pub fn claw_close(&mut self) {
        self.claw_state = ClawState::Moving;
        Self::motor_to_position(&mut self.claw_motor, config::CLAW_CLOSED_POS, config::CLAW_SPEED_PCT);
        self.claw_state = ClawState::Closed;
    }

    pub fn claw_toggle(&mut self) {
        println!("claw toggle from state {:?}, pos {:?}", self.claw_state, self.claw_motor.position().map(|a| a.as_degrees()).ok());
        match self.claw_state {
            ClawState::Open | ClawState::Moving => self.claw_close(),
            ClawState::Closed => self.claw_open(),
        }
    }

    pub async fn claw_grabbed(&mut self, timeout_ms: u64) -> bool {
        let start = Instant::now();
        let mut still = 0;
        while start.elapsed().as_millis() < timeout_ms as u128 {
            let vel = self.claw_motor.velocity().unwrap_or(0.0).abs();
            if vel < 3.0 {
                still += 1;
                if still >= 4 {
                    break;
                }
            } else {
                still = 0;
            }
            sleep(core::time::Duration::from_millis(20)).await;
        }

        let pos = self.claw_motor.position().map(|a| a.as_degrees()).unwrap_or(0.0);
        pos < config::CLAW_CLOSED_POS - config::CLAW_EMPTY_MARGIN
    }

    pub fn claw_temp(&self) -> f64 {
        self.claw_motor.temperature().unwrap_or(0.0)
    }

    pub fn is_ok(&self) -> bool {
        self.claw_motor.position().is_ok()
    }
}
