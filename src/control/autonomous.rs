use vexide::prelude::*;
use crate::config;
use crate::field;
use crate::control::motion;
use crate::subsystems::drivetrain::Drivetrain;
use crate::subsystems::lift::Lift;
use crate::subsystems::odometry::OdometryContext;
use crate::subsystems::top_mechanism::TopMechanism;

fn wrap360(mut a: f64) -> f64 {
    while a < 0.0 {
        a += 360.0;
    }
    while a >= 360.0 {
        a -= 360.0;
    }
    a
}

pub async fn grab_pin(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    top_mech: &mut TopMechanism,
) -> bool {
    top_mech.claw_close().await;
    if top_mech.claw_grabbed(900).await {
        return true;
    }
    let nudges = [2.0, 2.0, -6.0];
    for n in nudges {
        top_mech.claw_open().await;
        sleep(core::time::Duration::from_millis(200)).await;
        motion::drive_distance(dt, ctx, n, 6.0, 1000.0).await;
        top_mech.claw_close().await;
        if top_mech.claw_grabbed(900).await {
            return true;
        }
    }
    top_mech.claw_open().await;
    sleep(core::time::Duration::from_millis(200)).await;
    false
}

pub async fn route_mid_wall(
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    lift: &mut Lift,
    top_mech: &mut TopMechanism,
    flip_x: bool,
    flip_y: bool,
    plan: i32,
) {
    const FULL_SPEED: f64 = 12.0;
    const GOAL_SPEED: f64 = 4.8;

    let f = config::FIELD_SIZE;
    let x_fn = |x: f64| if flip_x { f - x } else { x };
    let y_fn = |y: f64| if flip_y { f - y } else { y };
    let facing = if flip_x { 270.0 } else { 90.0 };
    let sense = if flip_x != flip_y { -1.0 } else { 1.0 };
    let turned = |deg: f64| wrap360(facing + sense * deg);

    motion::clear_fault();
    ctx.odom.set_field_pose(x_fn(config::ROBOT_LENGTH / 2.0), y_fn(field::MID), facing);
    ctx.mcl.set_pose(x_fn(config::ROBOT_LENGTH / 2.0), y_fn(field::MID), facing);

    let stop_short = config::CLAW_REACH;
    // Preload release: drive 9 cm (3.54") closer than normal stop_short
    let preload_stop_short = (stop_short - 9.0 / 2.54).max(0.0);

    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_GOAL_X), y_fn(field::MID), FULL_SPEED, 2000.0).await;
    motion::turn_to_heading(dt, ctx, turned(90.0), 10.0, 1500.0).await;

    motion::approach_goal(
        dt,
        ctx,
        x_fn(field::LEFT_GOAL_X),
        y_fn(field::LEFT_GOAL_LOW_Y + preload_stop_short),
        8.0,
        GOAL_SPEED,
        2500.0,
    ).await;
    top_mech.claw_open().await;
    sleep(core::time::Duration::from_millis(250)).await;

    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_GOAL_X), y_fn(field::MID), FULL_SPEED, 2000.0).await;
    if plan == 0 {
        return;
    }

    motion::turn_to_heading(dt, ctx, turned(0.0), 10.0, 1500.0).await;
    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_MID_PIN_X - stop_short), y_fn(field::MID), FULL_SPEED, 2000.0).await;
    let got2 = grab_pin(dt, ctx, top_mech).await;

    if got2 {
        motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_GOAL_X), y_fn(field::MID), FULL_SPEED, 2000.0).await;
        motion::turn_to_heading(dt, ctx, turned(90.0), 10.0, 1500.0).await;
        lift.move_to_position(config::LIFT_LOW_POS, 1200.0).await;

        motion::approach_goal(
            dt,
            ctx,
            x_fn(field::LEFT_GOAL_X),
            y_fn(field::LEFT_GOAL_LOW_Y + stop_short),
            8.0,
            GOAL_SPEED,
            2500.0,
        ).await;
        top_mech.claw_open().await;
        sleep(core::time::Duration::from_millis(250)).await;
    }

    if plan < 2 {
        return;
    }

    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_GOAL_X), y_fn(field::MID), FULL_SPEED, 2000.0).await;
    lift.move_to_position(config::LIFT_HOME_POS, 1200.0).await;

    motion::turn_to_heading(dt, ctx, turned(0.0), 10.0, 1500.0).await;
    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_MID_PIN_X), y_fn(field::MID), FULL_SPEED, 2000.0).await;

    motion::turn_to_heading(dt, ctx, turned(90.0), 10.0, 1500.0).await;
    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_MID_PIN_X), y_fn(field::LEFT_LOWER_PIN_Y + stop_short), FULL_SPEED, 2000.0).await;
    let got3 = grab_pin(dt, ctx, top_mech).await;

    motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_MID_PIN_X), y_fn(field::LEFT_LOWER_PIN_Y), GOAL_SPEED, 1500.0).await;

    let middle_stop = config::ROBOT_LENGTH / 2.0 + 5.5;
    let diag = stop_short / 2.0_f64.sqrt();

    if got3 {
        motion::turn_to_heading(dt, ctx, turned(-45.0), 10.0, 1500.0).await;
        lift.move_to_position(config::LIFT_SCORE_POS, 1500.0).await;
        motion::approach_goal(
            dt,
            ctx,
            x_fn(field::MID - diag),
            y_fn(field::MID - diag),
            8.0,
            GOAL_SPEED,
            3000.0,
        ).await;
        top_mech.claw_open().await;
        sleep(core::time::Duration::from_millis(300)).await;

        motion::drive_straight_to(dt, ctx, x_fn(field::LEFT_MID_PIN_X), y_fn(field::LEFT_LOWER_PIN_Y), FULL_SPEED, 2500.0).await;
    }

    lift.move_to_position(config::LIFT_HOME_POS, 1200.0).await;
    motion::turn_to_heading(dt, ctx, turned(0.0), 10.0, 1500.0).await;
    motion::drive_straight_to(dt, ctx, x_fn(field::PIN_A_X - stop_short), y_fn(field::MIDDLE_PINS_Y), FULL_SPEED, 2500.0).await;
    let got_a = grab_pin(dt, ctx, top_mech).await;

    if got_a {
        motion::drive_straight_to(dt, ctx, x_fn(field::MID), y_fn(field::MIDDLE_PINS_Y), FULL_SPEED, 2500.0).await;
        motion::turn_to_heading(dt, ctx, turned(-90.0), 10.0, 1500.0).await;
        lift.move_to_position(config::LIFT_SCORE_POS, 1500.0).await;
        motion::approach_goal(
            dt,
            ctx,
            x_fn(field::MID),
            y_fn(field::MID - stop_short),
            8.0,
            GOAL_SPEED,
            3000.0,
        ).await;
        top_mech.claw_open().await;
        sleep(core::time::Duration::from_millis(300)).await;

        motion::drive_straight_to(dt, ctx, x_fn(field::MID), y_fn(field::MIDDLE_PINS_Y), FULL_SPEED, 2500.0).await;
        lift.move_to_position(config::LIFT_HOME_POS, 1200.0).await;
        motion::turn_to_heading(dt, ctx, turned(0.0), 10.0, 1500.0).await;
    }

    motion::drive_straight_to(dt, ctx, x_fn(field::PIN_B_X - stop_short), y_fn(field::MIDDLE_PINS_Y), FULL_SPEED, 2500.0).await;
    let got_b = grab_pin(dt, ctx, top_mech).await;

    if got_b {
        motion::drive_straight_to(dt, ctx, x_fn(field::PIN_B_X), y_fn(field::MIDDLE_PINS_Y), GOAL_SPEED, 1500.0).await;
        motion::turn_to_heading(dt, ctx, turned(90.0), 10.0, 1500.0).await;
        lift.move_to_position(config::LIFT_LOW_POS, 1200.0).await;
        motion::approach_goal(
            dt,
            ctx,
            x_fn(field::GOAL4_X),
            y_fn(field::GOAL4_Y + stop_short),
            8.0,
            GOAL_SPEED,
            3000.0,
        ).await;
        top_mech.claw_open().await;
        sleep(core::time::Duration::from_millis(300)).await;
    }

    motion::drive_straight_to(dt, ctx, x_fn(field::PIN_B_X), y_fn(field::MIDDLE_PINS_Y), FULL_SPEED, 2500.0).await;
    lift.move_to_position(config::LIFT_HOME_POS, 1200.0).await;
    motion::turn_to_heading(dt, ctx, turned(180.0), 10.0, 1500.0).await;
    motion::drive_straight_to(dt, ctx, x_fn(field::MID), y_fn(field::MIDDLE_PINS_Y), FULL_SPEED, 2500.0).await;
    motion::turn_to_heading(dt, ctx, turned(-90.0), 10.0, 1500.0).await;
    motion::drive_straight_to(dt, ctx, x_fn(field::MID), y_fn(field::MID - middle_stop), FULL_SPEED, 2500.0).await;
    motion::stop(dt);
}

pub async fn run_route_by_index(
    index: usize,
    dt: &mut Drivetrain,
    ctx: &mut OdometryContext<'_>,
    lift: &mut Lift,
    top_mech: &mut TopMechanism,
) {
    match index {
        0 => route_mid_wall(dt, ctx, lift, top_mech, false, false, 1).await,
        1 => route_mid_wall(dt, ctx, lift, top_mech, true, true, 1).await,
        2 => route_mid_wall(dt, ctx, lift, top_mech, false, true, 1).await,
        3 => route_mid_wall(dt, ctx, lift, top_mech, true, false, 1).await,
        4 => route_mid_wall(dt, ctx, lift, top_mech, false, true, 0).await,
        5 => route_mid_wall(dt, ctx, lift, top_mech, false, false, 0).await,
        6 => {
            ctx.odom.reset(0.0, 0.0, 0.0);
            motion::calibrate_turn_scale(dt, ctx, 2.0).await;
        }
        7 => route_mid_wall(dt, ctx, lift, top_mech, false, false, 2).await,
        _ => route_mid_wall(dt, ctx, lift, top_mech, false, false, 0).await,
    }
}
