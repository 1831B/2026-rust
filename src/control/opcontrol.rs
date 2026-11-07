//! Operator control: arcade drive with exponential response, mechanism bindings, and controller LCD.

use std::time::Instant;
use vexide::prelude::*;
use crate::config;
use crate::error_system::ErrorSystem;
use crate::subsystems::drivetrain::Drivetrain;
use crate::subsystems::lift::Lift;
use crate::subsystems::odometry::Odometry;
use crate::subsystems::top_mechanism::TopMechanism;

const DEADZONE: f64 = 7.0;
const EXPO: f64 = 1.8;
const SLEW_RATE: f64 = 0.9; // 900 mV per step in C++ -> 0.9V

fn apply_expo(v: f64, expo: f64) -> f64 {
    let sign = if v < 0.0 { -1.0 } else { 1.0 };
    sign * (v.abs() / 127.0).powf(expo) * 127.0
}

fn slew(cur: f64, tgt: f64, max_d: f64) -> f64 {
    cur + (tgt - cur).clamp(-max_d, max_d)
}

pub struct DriverControl {
    pub match_start: Instant,
    pub alerted_30: bool,
    pub alerted_10: bool,
    pub left_out: f64,
    pub right_out: f64,
    pub lift_up_held: bool,
    pub lift_down_held: bool,
    pub ctrl_update_timer: Instant,
    pub ctrl_page: usize,
}

impl Default for DriverControl {
    fn default() -> Self {
        Self::new()
    }
}

impl DriverControl {
    pub fn new() -> Self {
        Self {
            match_start: Instant::now(),
            alerted_30: false,
            alerted_10: false,
            left_out: 0.0,
            right_out: 0.0,
            lift_up_held: false,
            lift_down_held: false,
            ctrl_update_timer: Instant::now(),
            ctrl_page: 0,
        }
    }

    pub fn reset_match_timer(&mut self) {
        self.match_start = Instant::now();
        self.alerted_30 = false;
        self.alerted_10 = false;
    }

    pub async fn update_step(
        &mut self,
        controller: &mut Controller,
        dt: &mut Drivetrain,
        lift: &mut Lift,
        top_mech: &mut TopMechanism,
        odom: &Odometry,
        errors: &mut ErrorSystem,
    ) {
        let state = match controller.state() {
            Ok(s) => s,
            Err(_) => return,
        };

        // 1. Drivetrain Arcade Control (Left Y = throttle, Right X = turn)
        let mut raw_throttle = state.left_stick.y_raw() as f64;
        let mut raw_turn = state.right_stick.x_raw() as f64;

        if raw_throttle.abs() < DEADZONE {
            raw_throttle = 0.0;
        }
        if raw_turn.abs() < DEADZONE {
            raw_turn = 0.0;
        }

        let throttle = apply_expo(raw_throttle, EXPO);
        let turn = apply_expo(raw_turn, EXPO);

        let mut mix_l = throttle + turn;
        let mut mix_r = throttle - turn;

        let peak = mix_l.abs().max(mix_r.abs());
        if peak > 127.0 {
            mix_l *= 127.0 / peak;
            mix_r *= 127.0 / peak;
        }

        let tgt_l = (mix_l / 127.0 * config::MAX_DRIVE_VOLTAGE).clamp(
            -config::MAX_DRIVE_VOLTAGE,
            config::MAX_DRIVE_VOLTAGE,
        );
        let tgt_r = (mix_r / 127.0 * config::MAX_DRIVE_VOLTAGE).clamp(
            -config::MAX_DRIVE_VOLTAGE,
            config::MAX_DRIVE_VOLTAGE,
        );

        if tgt_l == 0.0 && tgt_r == 0.0 {
            self.left_out = 0.0;
            self.right_out = 0.0;
        } else {
            self.left_out = slew(self.left_out, tgt_l, SLEW_RATE);
            self.right_out = slew(self.right_out, tgt_r, SLEW_RATE);
        }
        dt.drive_voltage(self.left_out, self.right_out);

        // 2. Claw Toggle (Button A)
        if state.button_a.is_now_pressed() {
            top_mech.claw_toggle();
        }

        // 3. Lift Manual Control
        let l1 = state.button_l1.is_pressed();
        let l2 = state.button_l2.is_pressed();

        if l1 && !self.lift_up_held {
            lift.move_up(config::LIFT_UP_VOLTAGE);
            self.lift_up_held = true;
        } else if !l1 && self.lift_up_held {
            lift.hold();
            self.lift_up_held = false;
        }

        if l2 && !self.lift_down_held {
            lift.move_down(config::LIFT_DOWN_VOLTAGE);
            self.lift_down_held = true;
        } else if !l2 && self.lift_down_held {
            lift.hold();
            self.lift_down_held = false;
        }

        // 4. Lift Presets (X = score, B = home)
        if state.button_x.is_now_pressed() {
            lift.move_to_position(config::LIFT_SCORE_POS, 2000.0).await;
        }
        if state.button_b.is_now_pressed() {
            lift.move_to_position(config::LIFT_HOME_POS, 2000.0).await;
        }

        // 5. Emergency Stop (L1 + R1 + L2)
        if l1 && state.button_r1.is_pressed() && l2 {
            dt.emergency_stop();
            lift.stop();
            let _ = controller.try_rumble("---");
            let _ = controller.try_set_text("!! E-STOP !!", 0, 0);
            sleep(core::time::Duration::from_millis(1500)).await;
        }

        lift.update();
        top_mech.update();

        // 6. Match Time Alerts
        let elapsed = self.match_start.elapsed();
        let remain = if elapsed < config::MATCH_DURATION {
            config::MATCH_DURATION - elapsed
        } else {
            core::time::Duration::ZERO
        };

        if !self.alerted_30 && remain <= config::ALERT_30S {
            let _ = controller.try_rumble("..");
            self.alerted_30 = true;
        }
        if !self.alerted_10 && remain <= config::ALERT_10S {
            let _ = controller.try_rumble("...");
            self.alerted_10 = true;
        }

        // 7. Controller Telemetry Cycling (every 400ms)
        if self.ctrl_update_timer.elapsed().as_millis() > 400 {
            self.ctrl_update_timer = Instant::now();
            let secs = remain.as_secs();
            let errs = errors.active_count();
            let crit_mark = if errors.has_critical() { "!" } else { "" };

            match self.ctrl_page {
                0 => {
                    let ln0 = format!("T:{:02}s ERR:{}{}", secs, errs, crit_mark);
                    let claw_str = match top_mech.claw_state {
                        crate::subsystems::top_mechanism::ClawState::Closed => "CLO",
                        _ => "OPN",
                    };
                    let ln1 = format!("Lft:{:.0} Claw:{}", lift.position(), claw_str);
                    let ln2 = format!("B:{:.0}%", vexide::battery::capacity() * 100.0);
                    let _ = controller.try_set_text(&ln0, 0, 0);
                    let _ = controller.try_set_text(&ln1, 1, 0);
                    let _ = controller.try_set_text(&ln2, 2, 0);
                }
                1 => {
                    let t0 = format!("DrvL:{:.0}C R:{:.0}C", dt.left_temp(), dt.right_temp());
                    let t1 = format!("LftL:{:.0}C R:{:.0}C", lift.left_temp(), lift.right_temp());
                    let t2 = format!("Claw:{:.0}C", top_mech.claw_temp());
                    let _ = controller.try_set_text(&t0, 0, 0);
                    let _ = controller.try_set_text(&t1, 1, 0);
                    let _ = controller.try_set_text(&t2, 2, 0);
                }
                2 => {
                    let pose = odom.get_pose();
                    let p0 = format!("X:{:.1} Y:{:.1}", pose.x, pose.y);
                    let p1 = format!("Hdg:{:.1}", pose.heading);
                    let p2 = format!("Batt:{:.2}V", vexide::battery::voltage());
                    let _ = controller.try_set_text(&p0, 0, 0);
                    let _ = controller.try_set_text(&p1, 1, 0);
                    let _ = controller.try_set_text(&p2, 2, 0);
                }
                _ => {}
            }
            self.ctrl_page = (self.ctrl_page + 1) % 3;
        }
    }
}
