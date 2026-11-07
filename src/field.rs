//! Field coordinates and goal definitions.

pub const fn at(grid: f64) -> f64 {
    if grid <= -1700.0 {
        1.58
    } else if grid <= -1100.0 {
        23.11
    } else if grid <= -500.0 {
        46.66
    } else if grid <= 500.0 {
        70.20
    } else if grid <= 1100.0 {
        93.75
    } else if grid <= 1500.0 {
        117.30
    } else {
        138.83
    }
}

pub const MID: f64 = at(0.0);

pub const LEFT_GOAL_X: f64 = at(-1200.0);
pub const LEFT_GOAL_LOW_Y: f64 = at(-600.0);
pub const LEFT_GOAL_HIGH_Y: f64 = at(600.0);

pub const LEFT_MID_PIN_X: f64 = at(-600.0);
pub const LEFT_LOWER_PIN_Y: f64 = at(-600.0);

pub const PIN_A_X: f64 = at(0.0);
pub const PIN_B_X: f64 = at(600.0);
pub const MIDDLE_PINS_Y: f64 = at(-600.0);
pub const GOAL4_X: f64 = at(600.0);
pub const GOAL4_Y: f64 = at(-1200.0);

#[derive(Debug, Clone, Copy)]
pub struct GoalSpot {
    pub x: f64,
    pub y: f64,
    pub tier: i32,
}

pub const GOALS: [GoalSpot; 9] = [
    GoalSpot { x: at(0.0), y: at(0.0), tier: 2 },
    GoalSpot { x: at(-600.0), y: at(1200.0), tier: 1 },
    GoalSpot { x: at(-1200.0), y: at(600.0), tier: 1 },
    GoalSpot { x: at(1200.0), y: at(-600.0), tier: 1 },
    GoalSpot { x: at(600.0), y: at(-1200.0), tier: 1 },
    GoalSpot { x: at(-1200.0), y: at(-600.0), tier: 0 },
    GoalSpot { x: at(-600.0), y: at(-1200.0), tier: 0 },
    GoalSpot { x: at(600.0), y: at(1200.0), tier: 0 },
    GoalSpot { x: at(1200.0), y: at(600.0), tier: 0 },
];
