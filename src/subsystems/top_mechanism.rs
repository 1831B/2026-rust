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
    /// Target position in degrees for the current move, used to detect arrival
    claw_target_deg: f64,
}

impl TopMechanism {
    pub fn new(mut claw_motor: Motor) -> Self {
        let _ = claw_motor.reset_position();
        let _ = claw_motor.brake(BrakeMode::Hold);

        Self {
            claw_motor,
            claw_state: ClawState::Open,
            claw_target_deg: config::CLAW_OPEN_POS,
        }
    }

    fn motor_to_position(m: &mut Motor, deg: f64, speed_pct: f64) {
        // Green gearset = 200 RPM max; scale speed_pct (0–100) to RPM
        let rpm = ((speed_pct / 100.0) * 200.0) as i32;
        let _ = m.set_position_target(Angle::from_degrees(deg), rpm);
    }

    /// Call every loop tick (opcontrol or auto) to detect claw arrival and re-engage Hold.
    pub fn update(&mut self) {
        if self.claw_state == ClawState::Moving {
            let pos = self.claw_motor.position()
                .map(|a| a.as_degrees())
                .unwrap_or(self.claw_target_deg);
            let vel = self.claw_motor.velocity().unwrap_or(100.0).abs();
            let near = (pos - self.claw_target_deg).abs() < 8.0;
            let slow = vel < 5.0;

            if near && slow {
                // Arrived — switch to Hold so the claw grips/stays in place
                let _ = self.claw_motor.brake(BrakeMode::Hold);
                self.claw_state = if self.claw_target_deg == config::CLAW_OPEN_POS {
                    ClawState::Open
                } else {
                    ClawState::Closed
                };
            }
        }
    }

    /// Non-blocking: starts moving the claw and returns immediately.
    /// Used by opcontrol toggle (completion detected via `update()` each tick).
    fn claw_open_start(&mut self) {
        self.claw_state = ClawState::Moving;
        self.claw_target_deg = config::CLAW_OPEN_POS;
        Self::motor_to_position(&mut self.claw_motor, config::CLAW_OPEN_POS, config::CLAW_SPEED_PCT);
    }

    fn claw_close_start(&mut self) {
        self.claw_state = ClawState::Moving;
        self.claw_target_deg = config::CLAW_CLOSED_POS;
        Self::motor_to_position(&mut self.claw_motor, config::CLAW_CLOSED_POS, config::CLAW_SPEED_PCT);
    }

    /// Async: starts the move and waits until the motor arrives, then applies Hold.
    /// Use this in autonomous routines.
    pub async fn claw_open(&mut self) {
        self.claw_open_start();
        self.wait_for_claw(config::CLAW_OPEN_POS, 1500).await;
    }

    pub async fn claw_close(&mut self) {
        self.claw_close_start();
        self.wait_for_claw(config::CLAW_CLOSED_POS, 1500).await;
    }

    /// Waits until the claw motor reaches `target_deg` (or times out), then holds.
    async fn wait_for_claw(&mut self, target_deg: f64, timeout_ms: u64) {
        let start = Instant::now();
        while start.elapsed().as_millis() < timeout_ms as u128 {
            let pos = self.claw_motor.position()
                .map(|a| a.as_degrees())
                .unwrap_or(target_deg);
            let vel = self.claw_motor.velocity().unwrap_or(100.0).abs();
            if (pos - target_deg).abs() < 8.0 && vel < 5.0 {
                break;
            }
            sleep(core::time::Duration::from_millis(20)).await;
        }
        let _ = self.claw_motor.brake(BrakeMode::Hold);
        self.claw_state = if target_deg == config::CLAW_OPEN_POS {
            ClawState::Open
        } else {
            ClawState::Closed
        };
    }

    /// Non-blocking toggle for opcontrol (completion detected via `update()` each tick).
    pub fn claw_toggle(&mut self) {
        println!("claw toggle from state {:?}, pos {:?}", self.claw_state, self.claw_motor.position().map(|a| a.as_degrees()).ok());
        match self.claw_state {
            ClawState::Open | ClawState::Moving => self.claw_close_start(),
            ClawState::Closed => self.claw_open_start(),
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
