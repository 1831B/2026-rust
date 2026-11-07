//! Intake, claw, and wrist (flip) top mechanism subsystem.

use vexide::prelude::*;
use vexide::smart::motor::BrakeMode;
use crate::config;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntakeState {
    Idle,
    Intaking,
    Reversing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClawState {
    Open,
    Closed,
    Moving,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlipState {
    Front,
    Back,
    Moving,
}

pub struct TopMechanism {
    pub intake_motor: Motor,
    pub claw_motor: Motor,
    pub wrist_motor: Motor,

    pub intake_state: IntakeState,
    pub claw_state: ClawState,
    pub flip_state: FlipState,
    pub object_detected: bool,
}

impl TopMechanism {
    pub fn new(
        mut intake_motor: Motor,
        mut claw_motor: Motor,
        mut wrist_motor: Motor,
    ) -> Self {
        let _ = intake_motor.brake(BrakeMode::Coast);

        let _ = claw_motor.reset_position();
        let _ = claw_motor.brake(BrakeMode::Hold);

        let _ = wrist_motor.reset_position();
        let _ = wrist_motor.brake(BrakeMode::Hold);

        Self {
            intake_motor,
            claw_motor,
            wrist_motor,
            intake_state: IntakeState::Idle,
            claw_state: ClawState::Open,
            flip_state: FlipState::Front,
            object_detected: false,
        }
    }

    fn motor_to_position(m: &mut Motor, deg: f64, speed_pct: f64) {
        // Green gearset: 200 max RPM
        let rpm = (speed_pct * 2.0) as i32;
        let _ = m.set_position_target(Angle::from_degrees(deg), rpm);
    }

    pub fn update(&mut self) {
        if self.intake_state == IntakeState::Intaking {
            if let Ok(current_a) = self.intake_motor.current() {
                // 1.8 Amps corresponds to 1800 mA threshold in C++
                if current_a > 1.8 {
                    self.object_detected = true;
                    self.intake_stop();
                }
            }
        }
        if self.intake_state != IntakeState::Intaking {
            self.object_detected = false;
        }
    }

    pub fn intake_run(&mut self, voltage: f64) {
        self.intake_state = IntakeState::Intaking;
        self.object_detected = false;
        let _ = self.intake_motor.brake(BrakeMode::Coast);
        let _ = self.intake_motor.set_voltage(voltage);
    }

    pub fn intake_reverse(&mut self, voltage: f64) {
        self.intake_state = IntakeState::Reversing;
        let _ = self.intake_motor.set_voltage(-voltage);
    }

    pub fn intake_stop(&mut self) {
        self.intake_state = IntakeState::Idle;
        let _ = self.intake_motor.set_voltage(0.0);
        let _ = self.intake_motor.brake(BrakeMode::Coast);
    }

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
        match self.claw_state {
            ClawState::Open | ClawState::Moving => self.claw_close(),
            ClawState::Closed => self.claw_open(),
        }
    }

    pub fn flip_to_front(&mut self) {
        self.flip_state = FlipState::Moving;
        Self::motor_to_position(&mut self.wrist_motor, config::WRIST_FRONT_POS, config::WRIST_SPEED_PCT);
        self.flip_state = FlipState::Front;
    }

    pub fn flip_to_back(&mut self) {
        self.flip_state = FlipState::Moving;
        Self::motor_to_position(&mut self.wrist_motor, config::WRIST_BACK_POS, config::WRIST_SPEED_PCT);
        self.flip_state = FlipState::Back;
    }

    pub fn flip_toggle(&mut self) {
        match self.flip_state {
            FlipState::Front | FlipState::Moving => self.flip_to_back(),
            FlipState::Back => self.flip_to_front(),
        }
    }

    pub fn has_game_object(&self) -> bool {
        self.object_detected
    }

    pub fn intake_temp(&self) -> f64 {
        self.intake_motor.temperature().unwrap_or(0.0)
    }

    pub fn claw_temp(&self) -> f64 {
        self.claw_motor.temperature().unwrap_or(0.0)
    }

    pub fn wrist_temp(&self) -> f64 {
        self.wrist_motor.temperature().unwrap_or(0.0)
    }

    pub fn is_ok(&self) -> (bool, bool, bool) {
        (
            self.wrist_motor.position().is_ok(),
            self.intake_motor.position().is_ok(),
            self.claw_motor.position().is_ok(),
        )
    }
}
