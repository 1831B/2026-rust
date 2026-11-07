use std::f64::consts::PI;
use crate::config;
use crate::subsystems::odometry::{Motion, Pose, Status};

pub const NUM_PARTICLES: usize = 200;
pub const UPDATE_PERIOD_MS: u64 = 50;

const DEG2RAD: f64 = PI / 180.0;
const RAD2DEG: f64 = 180.0 / PI;

const ALPHA_FWD: f64 = 0.015;
const ALPHA_TURN: f64 = 0.012;
const ALPHA_DRIFT: f64 = 0.020;

const START_XY_SIGMA: f64 = 2.0;
const START_H_SIGMA: f64 = 1.0;
const ROUGHEN_XY: f64 = 0.35;

const HEADING_SIGMA: f64 = 3.0;

const SENSOR_MIN_MM: f64 = 30.0;
const SENSOR_MAX_MM: f64 = 1900.0;
const OUTLIER_FLOOR: f64 = 0.05;
const MAX_TURN_RATE: f64 = 45.0;

const CORRECTION_GAIN: f64 = 0.15;
const MAX_STD_IN: f64 = 4.0;
const MAX_JUMP_IN: f64 = 12.0;
const MIN_UPDATES: i32 = 3;

#[derive(Debug, Clone, Copy)]
pub struct Particle {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub weight: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Correction {
    pub valid: bool,
    pub dx: f64,
    pub dy: f64,
}

struct SensorGeom {
    bx: f64,
    by: f64,
    sin_f: f64,
    cos_f: f64,
}

const SENSORS: [SensorGeom; 3] = [
    SensorGeom {
        bx: config::DIST_LEFT_X,
        by: config::DIST_LEFT_Y,
        sin_f: -1.0,
        cos_f: 0.0,
    },
    SensorGeom {
        bx: config::DIST_RIGHT_X,
        by: config::DIST_RIGHT_Y,
        sin_f: 1.0,
        cos_f: 0.0,
    },
    SensorGeom {
        bx: config::DIST_FRONT_X,
        by: config::DIST_FRONT_Y,
        sin_f: 0.0,
        cos_f: 1.0,
    },
];

pub struct Mcl {
    particles: [Particle; NUM_PARTICLES],
    pub fusion_enabled: bool,
    updates_since_set: i32,
    best_x: f64,
    best_y: f64,
    best_h: f64,
    std_x: f64,
    std_y: f64,
    rng_state: u32,
}

fn rng_next(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

fn rng_uniform(state: &mut u32) -> f64 {
    (rng_next(state) & 0x7fffffff) as f64 / 0x7fffffff as f64
}

fn rng_gaussian(state: &mut u32, sigma: f64) -> f64 {
    if sigma < 1e-9 {
        return 0.0;
    }
    let u1 = rng_uniform(state) + 1e-12;
    let u2 = rng_uniform(state);
    sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
}

impl Default for Mcl {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl Mcl {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        let mut mcl = Self {
            particles: [Particle { x: 0.0, y: 0.0, heading: 0.0, weight: 1.0 / NUM_PARTICLES as f64 }; NUM_PARTICLES],
            fusion_enabled: true,
            updates_since_set: 0,
            best_x: x,
            best_y: y,
            best_h: heading,
            std_x: 0.0,
            std_y: 0.0,
            rng_state: 0xdeadbeef,
        };
        mcl.set_pose(x, y, heading);
        mcl
    }

    fn wrap360(mut a: f64) -> f64 {
        a %= 360.0;
        if a < 0.0 {
            a += 360.0;
        }
        a
    }

    fn angle_diff(a: f64, b: f64) -> f64 {
        (b - a + 540.0) % 360.0 - 180.0
    }

    fn distance_to_wall(sx: f64, sy: f64, dx: f64, dy: f64) -> f64 {
        let mut t: f64 = 1e9;
        if dx > 1e-9 {
            t = t.min((config::FIELD_SIZE - sx) / dx);
        }
        if dx < -1e-9 {
            t = t.min((0.0 - sx) / dx);
        }
        if dy > 1e-9 {
            t = t.min((config::FIELD_SIZE - sy) / dy);
        }
        if dy < -1e-9 {
            t = t.min((0.0 - sy) / dy);
        }
        t.max(0.0)
    }

    fn estimate(&mut self) {
        let mut sx = 0.0;
        let mut sy = 0.0;
        let mut sxx = 0.0;
        let mut syy = 0.0;
        let mut s_sin = 0.0;
        let mut s_cos = 0.0;

        for p in &self.particles {
            sx += p.weight * p.x;
            sy += p.weight * p.y;
            sxx += p.weight * p.x * p.x;
            syy += p.weight * p.y * p.y;
            s_sin += p.weight * (p.heading * DEG2RAD).sin();
            s_cos += p.weight * (p.heading * DEG2RAD).cos();
        }

        self.best_x = sx;
        self.best_y = sy;
        self.best_h = Self::wrap360(s_sin.atan2(s_cos) * RAD2DEG);
        self.std_x = (sxx - sx * sx).max(0.0).sqrt();
        self.std_y = (syy - sy * sy).max(0.0).sqrt();
    }

    pub fn set_pose(&mut self, x: f64, y: f64, heading: f64) {
        for p in &mut self.particles {
            p.x = x + rng_gaussian(&mut self.rng_state, START_XY_SIGMA);
            p.y = y + rng_gaussian(&mut self.rng_state, START_XY_SIGMA);
            p.heading = Self::wrap360(heading + rng_gaussian(&mut self.rng_state, START_H_SIGMA));
            p.weight = 1.0 / NUM_PARTICLES as f64;
        }
        self.updates_since_set = 0;
        self.estimate();
    }

    pub fn update(
        &mut self,
        motion: &Motion,
        odom_pose: &Pose,
        status: &Status,
        sensor_readings_mm: [Option<f64>; 3],
    ) -> Correction {
        let mut out = Correction::default();
        if !status.anchored {
            return out;
        }

        let abs_fwd = motion.forward.abs();
        let abs_turn = motion.turn.abs();
        for p in &mut self.particles {
            let dh = motion.turn + rng_gaussian(&mut self.rng_state, ALPHA_TURN * abs_turn + ALPHA_DRIFT * abs_fwd);
            let df = motion.forward + rng_gaussian(&mut self.rng_state, ALPHA_FWD * abs_fwd);
            let h_mid = (p.heading + dh / 2.0) * DEG2RAD;
            p.x += df * h_mid.sin();
            p.y += df * h_mid.cos();
            p.heading = Self::wrap360(p.heading + dh);
        }

        let mut meas_in = [0.0; 3];
        let mut have = [false; 3];
        let mut any = false;
        for i in 0..3 {
            if let Some(mm) = sensor_readings_mm[i] {
                if mm >= SENSOR_MIN_MM && mm <= SENSOR_MAX_MM {
                    meas_in[i] = mm / 25.4;
                    have[i] = true;
                    any = true;
                }
            }
        }

        if any && status.turn_rate < MAX_TURN_RATE {
            let mut total = 0.0;
            for p in &mut self.particles {
                let th = p.heading * DEG2RAD;
                let s = th.sin();
                let c = th.cos();

                let herr = Self::angle_diff(p.heading, odom_pose.heading) / HEADING_SIGMA;
                let mut w = (-0.5 * herr * herr).exp();

                for i in 0..3 {
                    if !have[i] {
                        continue;
                    }
                    let sn = &SENSORS[i];
                    let wx = p.x + sn.bx * c + sn.by * s;
                    let wy = p.y - sn.bx * s + sn.by * c;
                    let dx = s * sn.cos_f + c * sn.sin_f;
                    let dy = c * sn.cos_f - s * sn.sin_f;

                    let expected = Self::distance_to_wall(wx, wy, dx, dy);
                    let sigma = 1.0 + 0.03 * meas_in[i] + 0.03 * status.speed;
                    let e = (meas_in[i] - expected) / sigma;
                    w *= (-0.5 * e * e).exp() + OUTLIER_FLOOR;
                }

                p.weight = (p.weight * w).max(1e-300);
                total += p.weight;
            }

            for p in &mut self.particles {
                p.weight /= total;
            }

            let mut sq = 0.0;
            for p in &self.particles {
                sq += p.weight * p.weight;
            }
            let n_eff = 1.0 / sq;

            if n_eff < NUM_PARTICLES as f64 / 2.0 {
                let mut next = [Particle { x: 0.0, y: 0.0, heading: 0.0, weight: 0.0 }; NUM_PARTICLES];
                let step = 1.0 / NUM_PARTICLES as f64;
                let r = rng_uniform(&mut self.rng_state) * step;
                let mut cw = self.particles[0].weight;
                let mut j = 0;

                for i in 0..NUM_PARTICLES {
                    let u = r + i as f64 * step;
                    while u > cw && j < NUM_PARTICLES - 1 {
                        j += 1;
                        cw += self.particles[j].weight;
                    }
                    next[i] = self.particles[j];
                    next[i].weight = step;

                    let rx = rng_gaussian(&mut self.rng_state, ROUGHEN_XY);
                    let ry = rng_gaussian(&mut self.rng_state, ROUGHEN_XY);
                    next[i].x += rx;
                    next[i].y += ry;
                }
                self.particles = next;
            }
            self.updates_since_set += 1;
        }

        self.estimate();

        if self.fusion_enabled && self.updates_since_set >= MIN_UPDATES {
            let ex = self.best_x - odom_pose.x;
            let ey = self.best_y - odom_pose.y;
            if self.std_x < MAX_STD_IN && ex.abs() <= MAX_JUMP_IN {
                out.dx = CORRECTION_GAIN * ex;
                out.valid = true;
            }
            if self.std_y < MAX_STD_IN && ey.abs() <= MAX_JUMP_IN {
                out.dy = CORRECTION_GAIN * ey;
                out.valid = true;
            }
        }
        out
    }

    pub fn get_pose(&self) -> Pose {
        Pose {
            x: self.best_x,
            y: self.best_y,
            heading: self.best_h,
        }
    }

    pub fn get_uncertainty(&self) -> f64 {
        ((self.std_x * self.std_x + self.std_y * self.std_y) / 100.0).min(1.0)
    }

    pub fn get_particles(&self) -> &[Particle; NUM_PARTICLES] {
        &self.particles
    }
}
