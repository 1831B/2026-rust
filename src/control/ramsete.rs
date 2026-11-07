use std::f64::consts::PI;
use std::time::Instant;
use vexide::prelude::*;
use crate::config;
use crate::subsystems::drivetrain::Drivetrain;
use crate::subsystems::odometry::OdometryContext;

#[derive(Debug, Clone, Copy, Default)]
pub struct TrajectoryPoint {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub v: f64,
    pub omega: f64,
    pub t: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct WheelVelocities {
    pub left: f64,
    pub right: f64,
}

fn wrap_pi(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

fn compass_to_trig_rad(compass_deg: f64) -> f64 {
    wrap_pi((90.0 - compass_deg) * PI / 180.0)
}

pub fn compute(
    des: &TrajectoryPoint,
    x: f64,
    y: f64,
    heading_deg: f64,
    b: f64,
    zeta: f64,
) -> WheelVelocities {
    let theta_d = compass_to_trig_rad(des.heading);
    let theta = compass_to_trig_rad(heading_deg);

    let ex = theta.cos() * (des.x - x) + theta.sin() * (des.y - y);
    let ey = -theta.sin() * (des.x - x) + theta.cos() * (des.y - y);
    let et = wrap_pi(theta_d - theta);

    let k = 2.0 * zeta * (des.omega * des.omega + b * des.v * des.v).sqrt();
    let sinc_et = if et.abs() < 1e-6 { 1.0 } else { et.sin() / et };

    let v_cmd = des.v * et.cos() + k * ex;
    let w_cmd = des.omega + k * des.v * sinc_et * ey + k * et;

    let half_track = config::TRACK_WIDTH / 2.0;
    let left_vel = v_cmd - w_cmd * half_track;
    let right_vel = v_cmd + w_cmd * half_track;

    WheelVelocities {
        left: left_vel,
        right: right_vel,
    }
}

fn max_wheel_speed() -> f64 {
    (600.0 / 60.0) * PI * config::DRIVE_WHEEL_DIAMETER * config::DRIVE_GEAR_RATIO
}

fn vel_to_voltage(vel_in_s: f64, max_voltage: f64) -> f64 {
    let ratio = vel_in_s / max_wheel_speed();
    (ratio * max_voltage).clamp(-max_voltage, max_voltage)
}

pub async fn follow(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    path: &[TrajectoryPoint],
    max_voltage: f64,
    timeout_ms: f64,
) {
    if path.is_empty() {
        return;
    }

    let t0 = Instant::now();
    let start_t = path[0].t;
    let mut idx = 0;

    loop {
        ctx.step(dt);
        let elapsed_s = t0.elapsed().as_secs_f64();

        while idx + 1 < path.len() && path[idx + 1].t - start_t <= elapsed_s {
            idx += 1;
        }

        let mut setpoint = path[idx];
        if idx + 1 < path.len() {
            let a = &path[idx];
            let b_pt = &path[idx + 1];
            let seg = b_pt.t - a.t;
            let alpha = if seg > 1e-9 {
                ((elapsed_s - (a.t - start_t)) / seg).clamp(0.0, 1.0)
            } else {
                1.0
            };

            setpoint.x = a.x + alpha * (b_pt.x - a.x);
            setpoint.y = a.y + alpha * (b_pt.y - a.y);

            let mut dh = b_pt.heading - a.heading;
            while dh > 180.0 {
                dh -= 360.0;
            }
            while dh < -180.0 {
                dh += 360.0;
            }
            setpoint.heading = a.heading + alpha * dh;
            setpoint.v = a.v + alpha * (b_pt.v - a.v);
            setpoint.omega = a.omega + alpha * (b_pt.omega - a.omega);
        }

        let pose = ctx.odom.get_pose();
        let wv = compute(
            &setpoint,
            pose.x,
            pose.y,
            pose.heading,
            config::RAMSETE_B,
            config::RAMSETE_ZETA,
        );

        let lv = vel_to_voltage(wv.left, max_voltage);
        let rv = vel_to_voltage(wv.right, max_voltage);
        dt.drive_voltage(lv, rv);

        let at_end = idx >= path.len() - 1;
        let dist_to_end = ((path.last().unwrap().x - pose.x).powi(2)
            + (path.last().unwrap().y - pose.y).powi(2))
        .sqrt();

        if (at_end && dist_to_end < 1.5) || t0.elapsed().as_millis() as f64 > timeout_ms {
            break;
        }
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();
}

pub fn make_linear(
    x0: f64,
    y0: f64,
    _h0: f64,
    x1: f64,
    y1: f64,
    cruise_speed: f64,
    dt: f64,
) -> std::vec::Vec<TrajectoryPoint> {
    let mut traj = std::vec::Vec::new();
    let dist = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
    let total = dist / cruise_speed;
    let steps = 1.max((total / dt) as usize);

    let dx = x1 - x0;
    let dy = y1 - y0;
    let hdg = (dx.atan2(dy) * 180.0 / PI + 360.0) % 360.0;

    for i in 0..=steps {
        let alpha = i as f64 / steps as f64;
        let mut pt = TrajectoryPoint::default();
        pt.x = x0 + alpha * (x1 - x0);
        pt.y = y0 + alpha * (y1 - y0);
        pt.heading = hdg;
        pt.v = cruise_speed;
        pt.omega = 0.0;
        pt.t = i as f64 * dt;
        traj.push(pt);
    }
    if let Some(last) = traj.last_mut() {
        last.v = 0.0;
    }
    traj
}

pub fn make_turn(
    x: f64,
    y: f64,
    h0: f64,
    h1: f64,
    omega_deg_s: f64,
    dt: f64,
) -> std::vec::Vec<TrajectoryPoint> {
    let mut traj = std::vec::Vec::new();
    let mut dh = h1 - h0;
    while dh > 180.0 {
        dh -= 360.0;
    }
    while dh < -180.0 {
        dh += 360.0;
    }

    let omega_rad = (omega_deg_s * PI / 180.0) * if dh < 0.0 { -1.0 } else { 1.0 };
    let total_time = dh.abs() / omega_deg_s;
    let steps = 1.max((total_time / dt) as usize);

    for i in 0..=steps {
        let alpha = i as f64 / steps as f64;
        let mut pt = TrajectoryPoint::default();
        pt.x = x;
        pt.y = y;
        pt.heading = h0 + alpha * dh;
        pt.v = 0.0;
        pt.omega = if i < steps { omega_rad } else { 0.0 };
        pt.t = i as f64 * dt;
        traj.push(pt);
    }
    traj
}
