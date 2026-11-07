//! Robot configuration constants.

use core::time::Duration;

pub const LEFT_FRONT_PORT: u8 = 1;
pub const LEFT_MID_PORT: u8 = 2;
pub const LEFT_BACK_PORT: u8 = 3;
pub const RIGHT_FRONT_PORT: u8 = 4;
pub const RIGHT_MID_PORT: u8 = 5;
pub const RIGHT_BACK_PORT: u8 = 6;

pub const LIFT_LEFT_PORT: u8 = 11;
pub const LIFT_RIGHT_PORT: u8 = 19;
pub const WRIST_PORT: u8 = 12;
pub const INTAKE_PORT: u8 = 13;
pub const CLAW_PORT: u8 = 14;

pub const IMU_PORT: u8 = 10;
pub const FORWARD_TRACKER_PORT: u8 = 8;

pub const DIST_LEFT_PORT: u8 = 17;
pub const DIST_RIGHT_PORT: u8 = 18;

pub const TRACK_WHEEL_DIAMETER: f64 = 2.0;
pub const FORWARD_TRACKER_OFFSET: f64 = 0.0;

pub const DRIVE_WHEEL_DIAMETER: f64 = 3.25;
pub const DRIVE_GEAR_RATIO: f64 = 36.0 / 48.0;
pub const TRACK_WIDTH: f64 = 12.5;

pub const DRIVE_KP: f64 = 4.5;
pub const DRIVE_KI: f64 = 0.03;
pub const DRIVE_KD: f64 = 28.0;

pub const TURN_KP: f64 = 3.8;
pub const TURN_KI: f64 = 0.02;
pub const TURN_KD: f64 = 22.0;

pub const SWING_KP: f64 = 5.0;
pub const SWING_KI: f64 = 0.0;
pub const SWING_KD: f64 = 20.0;

pub const MAX_DRIVE_VOLTAGE: f64 = 12.0;
pub const MAX_TURN_VOLTAGE: f64 = 10.0;
pub const DRIVE_SETTLE_ERROR: f64 = 0.5;
pub const TURN_SETTLE_ERROR: f64 = 0.5;

pub const DRIVE_SLEW: f64 = 1.2;
pub const TURN_SLEW: f64 = 0.9;

pub const LIFT_SCORE_POS: f64 = 320.0;
pub const LIFT_LOW_POS: f64 = 150.0;
pub const LIFT_HOME_POS: f64 = 0.0;
pub const LIFT_UP_VOLTAGE: f64 = 10.0;
pub const LIFT_DOWN_VOLTAGE: f64 = 8.0;

pub const WRIST_FRONT_POS: f64 = 0.0;
pub const WRIST_BACK_POS: f64 = 180.0;
pub const WRIST_SPEED_PCT: f64 = 60.0;

pub const CLAW_OPEN_POS: f64 = 0.0;
pub const CLAW_CLOSED_POS: f64 = 90.0;
pub const CLAW_SPEED_PCT: f64 = 50.0;

pub const DIST_WALL_THRESHOLD: f64 = 300.0;

pub const LOW_BATTERY_THRESHOLD: f64 = 11.0;
pub const ERROR_TIMEOUT: Duration = Duration::from_millis(5000);
pub const MATCH_DURATION: Duration = Duration::from_millis(105000);
pub const ALERT_30S: Duration = Duration::from_millis(30000);
pub const ALERT_10S: Duration = Duration::from_millis(10000);

pub const RAMSETE_B: f64 = 2.0;
pub const RAMSETE_ZETA: f64 = 0.7;
