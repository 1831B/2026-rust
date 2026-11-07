//! High-precision dual-IMU and tracking-wheel odometry system with field anchoring.

use std::f64::consts::PI;
use std::time::Instant;
use vexide::prelude::*;
use crate::config;
use crate::error_system::{ErrorCode, ErrorSystem, Severity};
use crate::localization::mcl::{Correction, Mcl};
use crate::subsystems::drivetrain::Drivetrain;

const RAD2DEG: f64 = 180.0 / PI;
const DEG2RAD: f64 = PI / 180.0;

const TICKS_PER_REV: f64 = 36000.0;
const WHEEL_CIRC: f64 = config::TRACK_WHEEL_DIAMETER * PI;
const DEG_PER_INCH: f64 =
    360.0 / (config::DRIVE_WHEEL_DIAMETER * PI * config::DRIVE_GEAR_RATIO);

const MOUNT_SAMPLES: i32 = 50;
const FLAT_MIN_UP_Z: f64 = 0.978;
const LEARN_TURN_DEG: f64 = 20.0;
const MISMATCH_BASE_DEG: f64 = 3.0;
const MISMATCH_SCALE: f64 = 0.03;

#[derive(Debug, Clone, Copy, Default)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Motion {
    pub forward: f64,
    pub turn: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Status {
    pub imu_present: [bool; 2],
    pub imu_flat: [bool; 2],
    pub imu_verified: [bool; 2],
    pub imu_heading: [f64; 2],
    pub imu_disagreement: f64,
    pub heading_from_imu: bool,
    pub anchored: bool,
    pub speed: f64,
    pub turn_rate: f64,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub forced_sign: i32,
    pub mount_known: bool,
    pub flat: bool,
    pub up: [f64; 3],
    pub bias: [f64; 3],
    pub sign: i32,
    pub verified: bool,

    pub n: i32,
    pub sum_a: [f64; 3],
    pub sum_g: [f64; 3],
    pub min_a: [f64; 3],
    pub max_a: [f64; 3],

    pub have_last: bool,
    pub last_rot: f64,
    pub cum: f64,

    pub corr_dot: f64,
    pub ref_sq: f64,
    pub ref_abs: f64,
}

impl Channel {
    pub fn new(forced_sign: i32) -> Self {
        Self {
            forced_sign,
            mount_known: false,
            flat: false,
            up: [0.0, 0.0, 1.0],
            bias: [0.0, 0.0, 0.0],
            sign: 1,
            verified: false,
            n: 0,
            sum_a: [0.0; 3],
            sum_g: [0.0; 3],
            min_a: [0.0; 3],
            max_a: [0.0; 3],
            have_last: false,
            last_rot: 0.0,
            cum: 0.0,
            corr_dot: 0.0,
            ref_sq: 0.0,
            ref_abs: 0.0,
        }
    }
}

pub struct Inc {
    pub valid: bool,
    pub raw: f64,
    pub inc: f64,
}

pub struct Odometry {
    pub pose_x: f64,
    pub pose_y: f64,
    pub pose_heading: f64,
    pub turn_scale: f64,

    pub channels: [Channel; 2],

    baseline_set: bool,
    last_time: Option<Instant>,
    prev_l: f64,
    prev_r: f64,
    prev_trk: f64,
    have_l: bool,
    have_r: bool,
    have_trk: bool,

    ref_heading: f64,
    heading: f64,
    cum_diff: f64,
    turn_total: f64,
    anchored: bool,
    heading_from_imu: bool,

    acc_fwd: f64,
    acc_turn: f64,

    speed_f: f64,
    turn_rate_f: f64,
    last_mcl: Option<Instant>,
}

impl Default for Odometry {
    fn default() -> Self {
        Self::new()
    }
}

impl Odometry {
    pub fn new() -> Self {
        Self {
            pose_x: 0.0,
            pose_y: 0.0,
            pose_heading: 0.0,
            turn_scale: config::TURN_SCALE,
            channels: [
                Channel::new(config::IMU1_YAW_SIGN),
                Channel::new(config::IMU2_YAW_SIGN),
            ],
            baseline_set: false,
            last_time: None,
            prev_l: 0.0,
            prev_r: 0.0,
            prev_trk: 0.0,
            have_l: false,
            have_r: false,
            have_trk: false,
            ref_heading: 0.0,
            heading: 0.0,
            cum_diff: 0.0,
            turn_total: 0.0,
            anchored: false,
            heading_from_imu: false,
            acc_fwd: 0.0,
            acc_turn: 0.0,
            speed_f: 0.0,
            turn_rate_f: 0.0,
            last_mcl: None,
        }
    }

    fn wrap360(mut a: f64) -> f64 {
        a %= 360.0;
        if a < 0.0 {
            a += 360.0;
        }
        a
    }

    fn norm3(v: &[f64; 3]) -> f64 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }

    pub fn reset(&mut self, x: f64, y: f64, h: f64) {
        self.reset_internal(x, y, h, false);
    }

    pub fn set_field_pose(&mut self, x: f64, y: f64, h: f64) {
        self.reset_internal(x, y, h, true);
    }

    fn reset_internal(&mut self, x: f64, y: f64, h: f64, anchor: bool) {
        self.pose_x = x;
        self.pose_y = y;
        self.heading = h;
        self.ref_heading = h;
        self.pose_heading = Self::wrap360(h);
        for c in &mut self.channels {
            c.cum = 0.0;
        }
        self.cum_diff = 0.0;
        self.turn_total = 0.0;
        self.acc_fwd = 0.0;
        self.acc_turn = 0.0;
        self.anchored = anchor;
    }

    pub fn is_field_anchored(&self) -> bool {
        self.anchored
    }

    pub fn recalibrate_imu_mounts(&mut self) {
        self.channels = [
            Channel::new(config::IMU1_YAW_SIGN),
            Channel::new(config::IMU2_YAW_SIGN),
        ];
    }

    pub fn sample_mount(
        c: &mut Channel,
        accel: Option<[f64; 3]>,
        gyro: Option<[f64; 3]>,
        still: bool,
    ) {
        let (a, g) = match (accel, gyro) {
            (Some(a), Some(g)) => (a, g),
            _ => {
                c.n = 0;
                return;
            }
        };

        let amag = Self::norm3(&a);
        if !still || amag < 0.85 || amag > 1.15 || Self::norm3(&g) > 3.0 {
            c.n = 0;
            return;
        }

        for k in 0..3 {
            if c.n == 0 {
                c.sum_a[k] = 0.0;
                c.sum_g[k] = 0.0;
                c.min_a[k] = a[k];
                c.max_a[k] = a[k];
            }
            c.sum_a[k] += a[k];
            c.sum_g[k] += g[k];
            c.min_a[k] = c.min_a[k].min(a[k]);
            c.max_a[k] = c.max_a[k].max(a[k]);
        }
        c.n += 1;
        if c.n < MOUNT_SAMPLES {
            return;
        }

        for k in 0..3 {
            if c.max_a[k] - c.min_a[k] > 0.08 {
                c.n = 0;
                return;
            }
        }

        let mean_a = [
            c.sum_a[0] / c.n as f64,
            c.sum_a[1] / c.n as f64,
            c.sum_a[2] / c.n as f64,
        ];
        let m = Self::norm3(&mean_a);
        for k in 0..3 {
            c.up[k] = mean_a[k] / m;
            c.bias[k] = c.sum_g[k] / c.n as f64;
        }
        c.flat = c.up[2] >= FLAT_MIN_UP_Z;

        if c.flat {
            c.verified = true;
        } else if c.forced_sign != 0 {
            c.sign = c.forced_sign;
            c.verified = true;
        } else {
            c.sign = 1;
            c.verified = false;
        }
        c.mount_known = true;
        c.n = 0;
    }

    pub fn read_inc(
        c: &mut Channel,
        rotation_deg: Option<f64>,
        gyro_rate: Option<[f64; 3]>,
        dt: f64,
    ) -> Inc {
        if !c.mount_known {
            return Inc { valid: false, raw: 0.0, inc: 0.0 };
        }

        if c.flat {
            let r = match rotation_deg {
                Some(r) => r,
                None => return Inc { valid: false, raw: 0.0, inc: 0.0 },
            };
            if !c.have_last {
                c.last_rot = r;
                c.have_last = true;
                return Inc { valid: true, raw: 0.0, inc: 0.0 };
            }
            let inc = r - c.last_rot;
            c.last_rot = r;
            return Inc { valid: true, raw: inc, inc };
        }

        let g = match gyro_rate {
            Some(g) => g,
            None => return Inc { valid: false, raw: 0.0, inc: 0.0 },
        };
        let rate = (g[0] - c.bias[0]) * c.up[0]
            + (g[1] - c.bias[1]) * c.up[1]
            + (g[2] - c.bias[2]) * c.up[2];
        let raw = rate * dt;
        Inc {
            valid: true,
            raw,
            inc: raw * c.sign as f64,
        }
    }

    pub fn update(
        &mut self,
        left_pos_deg: f64,
        right_pos_deg: f64,
        tracker_ticks: Option<f64>,
        imu1_accel: Option<[f64; 3]>,
        imu1_gyro: Option<[f64; 3]>,
        imu1_rot: Option<f64>,
        imu2_accel: Option<[f64; 3]>,
        imu2_gyro: Option<[f64; 3]>,
        imu2_rot: Option<f64>,
        errors: &mut ErrorSystem,
    ) {
        let now = Instant::now();

        let trk_valid = tracker_ticks.is_some();
        let trk_in = tracker_ticks
            .map(|t| (t / TICKS_PER_REV) * WHEEL_CIRC)
            .unwrap_or(0.0);

        if !self.baseline_set {
            self.prev_l = left_pos_deg;
            self.prev_r = right_pos_deg;
            self.have_l = left_pos_deg.is_finite();
            self.have_r = right_pos_deg.is_finite();
            self.prev_trk = trk_in;
            self.have_trk = trk_valid;
            self.last_time = Some(now);
            self.baseline_set = true;
            return;
        }

        let dt = match self.last_time {
            Some(t) => {
                let secs = now.duration_since(t).as_secs_f64();
                if secs <= 0.0 { 0.005 } else { secs.min(0.1) }
            }
            None => 0.005,
        };
        self.last_time = Some(now);

        let mut dl = 0.0;
        let mut dr = 0.0;
        if left_pos_deg.is_finite() {
            if self.have_l {
                dl = left_pos_deg - self.prev_l;
            }
            self.prev_l = left_pos_deg;
            self.have_l = true;
        }
        if right_pos_deg.is_finite() {
            if self.have_r {
                dr = right_pos_deg - self.prev_r;
            }
            self.prev_r = right_pos_deg;
            self.have_r = true;
        }

        let dl_in = dl / DEG_PER_INCH;
        let dr_in = dr / DEG_PER_INCH;
        let enc_fwd = (dl_in + dr_in) / 2.0;
        let enc_dh = (dl_in - dr_in) / config::TRACK_WIDTH * RAD2DEG;

        let mut d_tracker = enc_fwd;
        if trk_valid {
            if self.have_trk {
                d_tracker = trk_in - self.prev_trk;
            }
            self.prev_trk = trk_in;
            self.have_trk = true;
        } else {
            self.have_trk = false;
        }

        let still = (dl.abs() + dr.abs()) < 0.2;

        if !self.channels[0].mount_known {
            Self::sample_mount(&mut self.channels[0], imu1_accel, imu1_gyro, still);
        }
        if !self.channels[1].mount_known {
            Self::sample_mount(&mut self.channels[1], imu2_accel, imu2_gyro, still);
        }

        let inc0 = Self::read_inc(&mut self.channels[0], imu1_rot, imu1_gyro, dt);
        let inc1 = Self::read_inc(&mut self.channels[1], imu2_rot, imu2_gyro, dt);

        let incs = [inc0, inc1];
        let mut sum = 0.0;
        let mut n = 0;
        for i in 0..2 {
            if incs[i].valid {
                self.channels[i].cum += incs[i].inc;
            }
            if incs[i].valid && self.channels[i].verified {
                sum += incs[i].inc;
                n += 1;
            }
        }

        let mut fused_dh = if n > 0 { sum / n as f64 } else { enc_dh };
        if n > 0 {
            fused_dh *= self.turn_scale;
        }
        self.heading_from_imu = n > 0;

        // Auto sign learning if tilted
        for i in 0..2 {
            let c = &mut self.channels[i];
            if !incs[i].valid || c.verified {
                continue;
            }
            c.corr_dot += incs[i].raw * fused_dh;
            c.ref_sq += fused_dh * fused_dh;
            c.ref_abs += fused_dh.abs();
            if c.ref_abs >= LEARN_TURN_DEG {
                if c.corr_dot.abs() >= 0.4 * c.ref_sq {
                    let s = if c.corr_dot >= 0.0 { 1 } else { -1 };
                    if s != c.sign {
                        c.cum = -c.cum;
                        c.sign = s;
                    }
                    c.verified = true;
                } else {
                    c.corr_dot = 0.0;
                    c.ref_sq = 0.0;
                    c.ref_abs = 0.0;
                }
            }
        }

        // Check dual IMU agreement
        if incs[0].valid && incs[1].valid && self.channels[0].verified && self.channels[1].verified {
            self.cum_diff += incs[0].inc - incs[1].inc;
            self.turn_total += fused_dh.abs();
            if self.cum_diff.abs() > MISMATCH_BASE_DEG + MISMATCH_SCALE * self.turn_total {
                errors.set(
                    ErrorCode::ImuMismatch,
                    Severity::Warning,
                    "IMUs disagree on heading",
                );
            }
        }

        let dh_rad = fused_dh * DEG2RAD;
        let d_center = d_tracker + config::FORWARD_TRACKER_OFFSET * dh_rad;
        let chord = if dh_rad.abs() < 1e-6 {
            d_center
        } else {
            2.0 * (dh_rad / 2.0).sin() * (d_center / dh_rad)
        };

        let h_mid = (self.heading + fused_dh / 2.0) * DEG2RAD;
        self.pose_x += chord * h_mid.sin();
        self.pose_y += chord * h_mid.cos();

        self.heading += fused_dh;
        self.pose_heading = Self::wrap360(self.heading);

        self.acc_fwd += chord;
        self.acc_turn += fused_dh;

        const ALPHA: f64 = 0.2;
        self.speed_f += ALPHA * ((chord / dt).abs() - self.speed_f);
        self.turn_rate_f += ALPHA * ((fused_dh / dt).abs() - self.turn_rate_f);
    }

    pub fn step(
        &mut self,
        left_pos_deg: f64,
        right_pos_deg: f64,
        tracker_ticks: Option<f64>,
        imu1_accel: Option<[f64; 3]>,
        imu1_gyro: Option<[f64; 3]>,
        imu1_rot: Option<f64>,
        imu2_accel: Option<[f64; 3]>,
        imu2_gyro: Option<[f64; 3]>,
        imu2_rot: Option<f64>,
        sensor_readings_mm: [Option<f64>; 3],
        mcl: &mut Mcl,
        errors: &mut ErrorSystem,
    ) {
        self.update(
            left_pos_deg,
            right_pos_deg,
            tracker_ticks,
            imu1_accel,
            imu1_gyro,
            imu1_rot,
            imu2_accel,
            imu2_gyro,
            imu2_rot,
            errors,
        );

        let now = Instant::now();
        let should_mcl = match self.last_mcl {
            Some(t) => now.duration_since(t).as_millis() >= 50,
            None => true,
        };

        if should_mcl {
            self.last_mcl = Some(now);
            let m = self.take_motion();
            let st = self.get_status();
            let c: Correction = mcl.update(&m, &self.get_pose(), &st, sensor_readings_mm);
            if c.valid {
                self.apply_correction(c.dx, c.dy);
            }
        }
    }

    pub fn take_motion(&mut self) -> Motion {
        let m = Motion {
            forward: self.acc_fwd,
            turn: self.acc_turn,
        };
        self.acc_fwd = 0.0;
        self.acc_turn = 0.0;
        m
    }

    pub fn apply_correction(&mut self, dx: f64, dy: f64) {
        self.pose_x += dx;
        self.pose_y += dy;
    }

    pub fn get_pose(&self) -> Pose {
        Pose {
            x: self.pose_x,
            y: self.pose_y,
            heading: self.pose_heading,
        }
    }

    pub fn get_status(&self) -> Status {
        Status {
            imu_present: [
                self.channels[0].mount_known,
                self.channels[1].mount_known,
            ],
            imu_flat: [self.channels[0].flat, self.channels[1].flat],
            imu_verified: [self.channels[0].verified, self.channels[1].verified],
            imu_heading: [
                Self::wrap360(self.ref_heading + self.channels[0].cum),
                Self::wrap360(self.ref_heading + self.channels[1].cum),
            ],
            imu_disagreement: self.cum_diff,
            heading_from_imu: self.heading_from_imu,
            anchored: self.anchored,
            speed: self.speed_f,
            turn_rate: self.turn_rate_f,
        }
    }
}

/// Bundles the robot's sensors for unified reading and odometry stepping.
pub struct Sensors<'a> {
    pub imu: &'a InertialSensor,
    pub imu2: &'a InertialSensor,
    pub forward_tracker: &'a RotationSensor,
    pub dist_left: &'a DistanceSensor,
    pub dist_right: &'a DistanceSensor,
    pub dist_front: &'a DistanceSensor,
}

impl<'a> Sensors<'a> {
    pub fn new(
        imu: &'a InertialSensor,
        imu2: &'a InertialSensor,
        forward_tracker: &'a RotationSensor,
        dist_left: &'a DistanceSensor,
        dist_right: &'a DistanceSensor,
        dist_front: &'a DistanceSensor,
    ) -> Self {
        Self {
            imu,
            imu2,
            forward_tracker,
            dist_left,
            dist_right,
            dist_front,
        }
    }

    pub fn read_data(&self) -> (
        Option<f64>,
        Option<[f64; 3]>,
        Option<[f64; 3]>,
        Option<f64>,
        Option<[f64; 3]>,
        Option<[f64; 3]>,
        Option<f64>,
        [Option<f64>; 3],
    ) {
        let trk = self.forward_tracker.position().map(|a| a.as_degrees() * 100.0).ok();
        let a1 = self.imu.acceleration().map(|v| [v.x, v.y, v.z]).ok();
        let g1 = self.imu.gyro_rate().map(|v| [v.x, v.y, v.z]).ok();
        let r1 = self.imu.rotation().map(|a| a.as_degrees()).ok();
        let a2 = self.imu2.acceleration().map(|v| [v.x, v.y, v.z]).ok();
        let g2 = self.imu2.gyro_rate().map(|v| [v.x, v.y, v.z]).ok();
        let r2 = self.imu2.rotation().map(|a| a.as_degrees()).ok();
        let dl = self.dist_left.object().ok().flatten().map(|o| o.distance as f64);
        let dr = self.dist_right.object().ok().flatten().map(|o| o.distance as f64);
        let df = self.dist_front.object().ok().flatten().map(|o| o.distance as f64);
        (trk, a1, g1, r1, a2, g2, r2, [dl, dr, df])
    }

    pub fn step_odometry(
        &self,
        dt: &Drivetrain,
        odom: &mut Odometry,
        mcl: &mut Mcl,
        errors: &mut ErrorSystem,
    ) {
        let (trk, a1, g1, r1, a2, g2, r2, dists) = self.read_data();
        let lp = dt.left_position();
        let rp = dt.right_position();
        odom.step(lp, rp, trk, a1, g1, r1, a2, g2, r2, dists, mcl, errors);
        let imu1_ok = r1.is_some();
        let imu2_ok = r2.is_some();
        let trk_ok = trk.is_some();
        let batt_v = vexide::battery::voltage();
        let max_temp = dt.left_temp().max(dt.right_temp());
        errors.update_hardware(imu1_ok, imu2_ok, trk_ok, batt_v, max_temp);
    }
}

/// Context bundling odometry state and sensors for continuous updates during motion routines.
pub struct OdometryContext<'a> {
    pub odom: &'a mut Odometry,
    pub mcl: &'a mut Mcl,
    pub errors: &'a mut ErrorSystem,
    pub sensors: &'a Sensors<'a>,
}

impl<'a> OdometryContext<'a> {
    pub fn new(
        odom: &'a mut Odometry,
        mcl: &'a mut Mcl,
        errors: &'a mut ErrorSystem,
        sensors: &'a Sensors<'a>,
    ) -> Self {
        Self {
            odom,
            mcl,
            errors,
            sensors,
        }
    }

    #[inline]
    pub fn step(&mut self, dt: &Drivetrain) {
        self.sensors.step_odometry(dt, self.odom, self.mcl, self.errors);
    }
}
