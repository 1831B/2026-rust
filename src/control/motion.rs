use std::f64::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use vexide::prelude::*;
use crate::config;
use crate::error_system::{ErrorCode, Severity};
use crate::control::pid::Pid;
use crate::subsystems::drivetrain::Drivetrain;
use crate::subsystems::odometry::{Odometry, OdometryContext};

static FAULT: AtomicBool = AtomicBool::new(false);

pub fn faulted() -> bool {
    FAULT.load(Ordering::SeqCst)
}

pub fn clear_fault() {
    FAULT.store(false, Ordering::SeqCst);
}

fn raise_fault(dt: &mut Drivetrain, ctx: &mut OdometryContext<'_>, why: &str) {
    FAULT.store(true, Ordering::SeqCst);
    dt.brake();
    ctx.errors.set(ErrorCode::MotionFault, Severity::Critical, why);
    println!("MOTION FAULT: {}", why);
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

fn normalize_angle(mut a: f64) -> f64 {
    while a > 180.0 {
        a -= 360.0;
    }
    while a < -180.0 {
        a += 360.0;
    }
    a
}

fn angle_to_point(odom: &Odometry, tx: f64, ty: f64) -> f64 {
    let dx = tx - odom.pose_x;
    let dy = ty - odom.pose_y;
    let mut ang = dx.atan2(dy) * 180.0 / PI;
    if ang < 0.0 {
        ang += 360.0;
    }
    ang
}

fn dist_to_point(odom: &Odometry, tx: f64, ty: f64) -> f64 {
    let dx = tx - odom.pose_x;
    let dy = ty - odom.pose_y;
    (dx * dx + dy * dy).sqrt()
}

fn slew(cur: f64, tgt: f64, max_d: f64) -> f64 {
    cur + (tgt - cur).clamp(-max_d, max_d)
}

pub async fn drive_distance(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    inches: f64,
    max_voltage: f64,
    timeout_ms: f64,
) {
    if faulted() {
        return;
    }
    const DEG_PER_INCH: f64 =
        360.0 / (config::DRIVE_WHEEL_DIAMETER * PI * config::DRIVE_GEAR_RATIO);
    let mut drive_pid = Pid::new(
        config::DRIVE_KP,
        config::DRIVE_KI,
        config::DRIVE_KD,
        50.0,
        config::DRIVE_SETTLE_ERROR * DEG_PER_INCH,
        150.0,
    );
    let mut head_pid = Pid::new(
        config::TURN_KP * 0.5,
        0.0,
        config::TURN_KD * 0.5,
        20.0,
        1.0,
        100.0,
    );

    ctx.step(dt);
    let start_l = dt.left_position();
    let start_r = dt.right_position();
    let start_h = ctx.odom.pose_heading;

    let target_ticks = inches * DEG_PER_INCH;
    let mut lv = 0.0;
    let mut rv = 0.0;
    let t0 = Instant::now();
    let mut bad_since: Option<Instant> = None;

    loop {
        ctx.step(dt);
        let traveled = ((dt.left_position() - start_l) + (dt.right_position() - start_r)) / 2.0;
        let err = target_ticks - traveled;
        let h_err = normalize_angle(start_h - ctx.odom.pose_heading);

        if h_err.abs() > 60.0 {
            let since = *bad_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() > 200 {
                raise_fault(dt, ctx, "heading lost while driving");
                return;
            }
        } else {
            bad_since = None;
        }

        let drive_out_mv = drive_pid.update(err).clamp(-max_voltage * 1000.0, max_voltage * 1000.0);
        let head_out_mv = head_pid.update(h_err).clamp(-max_voltage * 400.0, max_voltage * 400.0);

        let drive_out = drive_out_mv / 1000.0;
        let head_out = head_out_mv / 1000.0;

        lv = slew(lv, drive_out + head_out, config::DRIVE_SLEW);
        rv = slew(rv, drive_out - head_out, config::DRIVE_SLEW);
        dt.drive_voltage(lv, rv);

        if drive_pid.is_settled() || t0.elapsed().as_millis() as f64 > timeout_ms {
            break;
        }
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();
}

pub async fn drive_straight_to(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    x: f64,
    y: f64,
    max_voltage: f64,
    timeout_ms: f64,
) {
    let h = ctx.odom.pose_heading * PI / 180.0;
    let d = (x - ctx.odom.pose_x) * h.sin() + (y - ctx.odom.pose_y) * h.cos();
    drive_distance(dt, ctx, d, max_voltage, timeout_ms).await;
}

pub async fn approach_goal(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    x: f64,
    y: f64,
    standoff: f64,
    max_voltage: f64,
    timeout_ms: f64,
) {
    let h = ctx.odom.pose_heading * PI / 180.0;
    let d = (x - ctx.odom.pose_x) * h.sin() + (y - ctx.odom.pose_y) * h.cos();

    if d <= standoff + 1.0 {
        drive_distance(dt, ctx, d, max_voltage, timeout_ms).await;
        return;
    }

    drive_distance(dt, ctx, d - standoff, max_voltage, timeout_ms).await;
    sleep(core::time::Duration::from_millis(120)).await;

    let mut sum = 0.0;
    let mut n = 0;
    for _ in 0..5 {
        ctx.step(dt);
        if let Ok(Some(obj)) = ctx.sensors.dist_front.object() {
            let mm = obj.distance as f64;
            if mm > 0.0 && mm < 1000.0 {
                sum += mm / 25.4;
                n += 1;
            }
        }
        sleep(core::time::Duration::from_millis(15)).await;
    }

    let mut remaining = standoff;
    if n >= 3 {
        let reading = sum / n as f64;
        let expected = standoff + config::GOAL_FRONT_READING;
        if (reading - expected).abs() < 3.0 {
            remaining = reading - config::GOAL_FRONT_READING;
        }
    }
    drive_distance(dt, ctx, remaining, max_voltage, timeout_ms).await;
}

pub async fn turn_to_heading(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    heading: f64,
    max_voltage: f64,
    timeout_ms: f64,
) {
    if faulted() {
        return;
    }
    let mut pid = Pid::new(
        config::TURN_KP,
        config::TURN_KI,
        config::TURN_KD,
        30.0,
        config::TURN_SETTLE_ERROR,
        100.0,
    );
    let mut lv = 0.0;
    let mut rv = 0.0;
    let t0 = Instant::now();
    let mut best_err = normalize_angle(heading - ctx.odom.pose_heading).abs();
    let mut runaway_since: Option<Instant> = None;
    let mut window_t = Instant::now();
    let mut window_h = ctx.odom.pose_heading;

    loop {
        ctx.step(dt);
        let err = normalize_angle(heading - ctx.odom.pose_heading);
        let abs_err = err.abs();
        best_err = best_err.min(abs_err);
        if abs_err > best_err + 25.0 && abs_err > 30.0 {
            let since = *runaway_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() > 150 {
                raise_fault(dt, ctx, "turn went the wrong way, check IMU");
                return;
            }
        } else {
            runaway_since = None;
        }
        if window_t.elapsed().as_millis() >= 500 {
            let moved = normalize_angle(ctx.odom.pose_heading - window_h).abs();
            if abs_err > 10.0 && moved < 1.0 {
                raise_fault(dt, ctx, "heading not changing while turning");
                return;
            }
            window_t = Instant::now();
            window_h = ctx.odom.pose_heading;
        }
        let mut out_mv = pid.update(err);
        out_mv += (config::TURN_MIN_VOLTAGE * 1000.0 * (err.abs() / 2.0).min(1.0)).copysign(err);
        out_mv = out_mv.clamp(-max_voltage * 1000.0, max_voltage * 1000.0);

        let out = out_mv / 1000.0;

        lv = slew(lv, out, config::TURN_SLEW);
        rv = slew(rv, -out, config::TURN_SLEW);
        dt.drive_voltage(lv, rv);

        if pid.is_settled() || t0.elapsed().as_millis() as f64 > timeout_ms {
            break;
        }
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();
}

pub async fn turn_to_point(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    x: f64,
    y: f64,
    max_voltage: f64,
    timeout_ms: f64,
) {
    let ang = angle_to_point(ctx.odom, x, y);
    turn_to_heading(dt, ctx, ang, max_voltage, timeout_ms).await;
}

pub async fn move_to_point(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    x: f64,
    y: f64,
    max_voltage: f64,
    timeout_ms: f64,
) {
    if faulted() {
        return;
    }
    turn_to_point(dt, ctx, x, y, max_voltage * 0.8, 2000.0).await;

    const DEG_PER_INCH: f64 =
        360.0 / (config::DRIVE_WHEEL_DIAMETER * PI * config::DRIVE_GEAR_RATIO);
    let mut drive_pid = Pid::new(
        config::DRIVE_KP,
        config::DRIVE_KI,
        config::DRIVE_KD,
        50.0 * DEG_PER_INCH,
        config::DRIVE_SETTLE_ERROR * DEG_PER_INCH,
        200.0,
    );
    let mut head_pid = Pid::new(
        config::TURN_KP,
        config::TURN_KI,
        config::TURN_KD,
        30.0,
        1.0,
        100.0,
    );

    let mut lv = 0.0;
    let mut rv = 0.0;
    let t0 = Instant::now();

    loop {
        ctx.step(dt);
        let d = dist_to_point(ctx.odom, x, y);
        let h_err = if d < 3.0 {
            0.0
        } else {
            normalize_angle(angle_to_point(ctx.odom, x, y) - ctx.odom.pose_heading)
        };

        let drive_out_mv = drive_pid.update(d * DEG_PER_INCH).clamp(-max_voltage * 1000.0, max_voltage * 1000.0);
        let head_out_mv = head_pid.update(h_err).clamp(-max_voltage * 500.0, max_voltage * 500.0);

        let drive_out = drive_out_mv / 1000.0;
        let head_out = head_out_mv / 1000.0;

        lv = slew(lv, drive_out + head_out, config::DRIVE_SLEW);
        rv = slew(rv, drive_out - head_out, config::DRIVE_SLEW);
        dt.drive_voltage(lv, rv);

        if drive_pid.is_settled() || t0.elapsed().as_millis() as f64 > timeout_ms {
            break;
        }
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();
}

pub async fn follow_path(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    path: &[Point],
    lookahead: f64,
    max_voltage: f64,
) {
    if faulted() {
        return;
    }
    if path.is_empty() {
        return;
    }

    let mut closest = 0;
    let t0 = Instant::now();

    loop {
        ctx.step(dt);
        let rx = ctx.odom.pose_x;
        let ry = ctx.odom.pose_y;

        let mut min_d = 1e9;
        for i in closest..path.len() {
            let d = ((path[i].x - rx).powi(2) + (path[i].y - ry).powi(2)).sqrt();
            if d < min_d {
                min_d = d;
                closest = i;
            }
        }

        let mut goal = *path.last().unwrap();
        for i in closest..path.len() {
            let d = ((path[i].x - rx).powi(2) + (path[i].y - ry).powi(2)).sqrt();
            if d >= lookahead {
                goal = path[i];
                break;
            }
        }

        let dist_end = ((path.last().unwrap().x - rx).powi(2) + (path.last().unwrap().y - ry).powi(2)).sqrt();
        if dist_end < 3.0 || t0.elapsed().as_millis() > 8000 {
            break;
        }

        let mut tgt_h = (goal.x - rx).atan2(goal.y - ry) * 180.0 / PI;
        if tgt_h < 0.0 {
            tgt_h += 360.0;
        }
        let h_err = normalize_angle(tgt_h - ctx.odom.pose_heading);
        let curvature = (2.0 * (h_err * PI / 180.0).sin()) / lookahead;

        let lv_t = max_voltage * (1.0 - curvature * config::TRACK_WIDTH / 2.0);
        let rv_t = max_voltage * (1.0 + curvature * config::TRACK_WIDTH / 2.0);
        let scale = max_voltage / lv_t.abs().max(rv_t.abs());

        dt.drive_voltage(lv_t * scale, rv_t * scale);
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();
}

pub async fn swing_turn(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    heading: f64,
    left_swing: bool,
    max_voltage: f64,
    timeout_ms: f64,
) {
    if faulted() {
        return;
    }
    let mut pid = Pid::new(
        config::SWING_KP,
        config::SWING_KI,
        config::SWING_KD,
        30.0,
        config::TURN_SETTLE_ERROR,
        200.0,
    );
    let t0 = Instant::now();

    loop {
        ctx.step(dt);
        let err = normalize_angle(heading - ctx.odom.pose_heading);
        let out_mv = pid.update(err).clamp(-max_voltage * 1000.0, max_voltage * 1000.0);
        let out = out_mv / 1000.0;

        if left_swing {
            dt.drive_voltage(out, 0.0);
        } else {
            dt.drive_voltage(0.0, -out);
        }

        if pid.is_settled() || t0.elapsed().as_millis() as f64 > timeout_ms {
            break;
        }
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();
}

pub async fn calibrate_turn_scale(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    revolutions: f64,
) -> f64 {
    let old_scale = ctx.odom.turn_scale;
    ctx.odom.turn_scale = 1.0;

    #[derive(Clone, Copy)]
    struct Sample {
        h: f64,
        d: f64,
    }
    let mut s: std::vec::Vec<Sample> = std::vec::Vec::new();
    let mut prev = ctx.odom.pose_heading;
    let mut unwrapped = 0.0;
    let t0 = Instant::now();
    let target = 360.0 * revolutions + 60.0;

    while unwrapped < target && t0.elapsed().as_millis() < 15000 {
        ctx.step(dt);
        let hnow = ctx.odom.pose_heading;
        unwrapped += normalize_angle(hnow - prev);
        prev = hnow;

        let ok_dist = ctx.sensors.dist_left.object().ok().flatten().and_then(|obj| {
            let mm = obj.distance as f64;
            if mm >= 30.0 && mm <= 1900.0 {
                Some(mm / 25.4)
            } else {
                None
            }
        });

        s.push(Sample {
            h: unwrapped,
            d: ok_dist.unwrap_or(1e9),
        });
        dt.drive_voltage(3.5, -3.5);
        sleep(core::time::Duration::from_millis(10)).await;
    }
    dt.brake();

    let mut mins: std::vec::Vec<f64> = std::vec::Vec::new();
    let n = s.len();
    let mut run0 = 0;
    for i in 1..=n {
        let brk = i == n || s[i].d > 1e8 || s[i - 1].d > 1e8 || (s[i].d - s[i - 1].d).abs() > 6.0;
        if !brk {
            continue;
        }
        let lo = run0;
        let hi = i - 1;
        run0 = i;
        if hi - lo < 5 || s[lo].d > 1e8 || s[hi].h - s[lo].h < 30.0 {
            continue;
        }

        let mut saa = 0.0;
        let mut sab = 0.0;
        let mut sbb = 0.0;
        let mut say = 0.0;
        let mut sby = 0.0;
        for k in lo..=hi {
            let r = s[k].h * PI / 180.0;
            let c = r.cos();
            let sn = r.sin();
            let y = 1.0 / s[k].d;
            saa += c * c;
            sab += c * sn;
            sbb += sn * sn;
            say += c * y;
            sby += sn * y;
        }

        let det = saa * sbb - sab * sab;
        if det.abs() < 1e-9 {
            continue;
        }
        let a_fit = (say * sbb - sby * sab) / det;
        let b_fit = (sby * saa - say * sab) / det;
        let amp = a_fit.hypot(b_fit);
        if amp < 1.0 / 90.0 || amp > 1.0 / 3.0 {
            continue;
        }

        let mut th0 = b_fit.atan2(a_fit) * 180.0 / PI;
        let centre = 0.5 * (s[lo].h + s[hi].h);
        th0 += 360.0 * ((centre - th0) / 360.0).round();
        if (th0 - centre).abs() > 45.0 {
            continue;
        }

        let mut sq = 0.0;
        for k in lo..=hi {
            let c = ((s[k].h - th0) * PI / 180.0).cos();
            let pred = if c > 0.2 { 1.0 / (amp * c) } else { 1e3 };
            sq += (pred - s[k].d).powi(2);
        }
        if (sq / (hi - lo + 1) as f64).sqrt() > 1.5 {
            continue;
        }
        mins.push(th0);
    }

    let mut scale = old_scale;
    let mut best_in = 0;
    let mut best: std::vec::Vec<f64> = std::vec::Vec::new();
    let m = mins.len();
    for i in 0..m {
        for j in (i + 1)..m {
            let span = mins[j] - mins[i];
            let steps = (span / 90.0).round();
            if steps < 4.0 {
                continue;
            }
            let sc = 90.0 * steps / span;
            if sc < 0.85 || sc > 1.15 {
                continue;
            }
            let mut in_group: std::vec::Vec<f64> = std::vec::Vec::new();
            for k in 0..m {
                let r = (mins[k] - mins[i]) * sc / 90.0;
                if ((r - r.round()) * 90.0).abs() < 2.0 {
                    in_group.push(mins[k]);
                }
            }
            if in_group.len() > best_in {
                best_in = in_group.len();
                best = in_group;
            }
        }
    }

    if best_in >= 4 {
        let span = best.last().unwrap() - best.first().unwrap();
        let sc0 = 90.0 * (span / 90.0).round() / span;
        let steps_f = (span * sc0 / 90.0).round();
        if steps_f >= 4.0 {
            scale = 90.0 * steps_f / span;
        }
    }
    ctx.odom.turn_scale = scale;
    scale
}

pub fn stop(dt: &mut Drivetrain) {
    dt.brake();
}
