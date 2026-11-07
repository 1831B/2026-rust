use vexide::color::Color;
use vexide::display::{Alignment, Display, Font, FontFamily, FontSize, Line, Rect, Text, TouchState};
use std::f64::consts::PI;
use std::time::Instant;
use crate::config;
use crate::error_system::{ErrorCode, ErrorSystem, Severity};
use crate::field;
use crate::robot_state::{Mode, RobotState};
use crate::control::auton_selector::AutonomousSelector;
use crate::subsystems::odometry::Odometry;
use crate::subsystems::drivetrain::Drivetrain;
use crate::subsystems::lift::Lift;
use crate::subsystems::top_mechanism::TopMechanism;

pub const SCR_W: i16 = 480;
pub const SCR_H: i16 = 240;
pub const TOP_H: i16 = 24;
pub const BOT_Y: i16 = 206;
pub const BODY_Y0: i16 = TOP_H + 6;
pub const BODY_Y1: i16 = BOT_Y - 4;

pub const C_BG: Color = Color::from_raw(0x000000);
pub const C_PANEL: Color = Color::from_raw(0x13110B);
pub const C_PANEL2: Color = Color::from_raw(0x262010);
pub const C_TRACK: Color = Color::from_raw(0x2E2813);
pub const C_GOLD: Color = Color::from_raw(0xFFC72C);
pub const C_GOLD_D: Color = Color::from_raw(0x7A5E10);
pub const C_WHITE: Color = Color::from_raw(0xF6F1E3);
pub const C_MUTED: Color = Color::from_raw(0x8C8670);
pub const C_GOOD: Color = Color::from_raw(0x34D27B);
pub const C_WARN: Color = Color::from_raw(0xFF9F0A);
pub const C_BAD: Color = Color::from_raw(0xFF453A);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    Boot = 0,
    Home,
    Systems,
    AutonSelect,
    Faults,
}

pub const PAGE_COUNT: usize = 4;
pub const PAGES: [Screen; PAGE_COUNT] = [Screen::Home, Screen::Systems, Screen::AutonSelect, Screen::Faults];
pub const PAGE_NAMES: [&str; PAGE_COUNT] = ["HOME", "SYSTEMS", "AUTON", "FAULTS"];
pub const TAB_W: i16 = 100;
pub const TAB_GAP: i16 = 10;
pub const TRAIL_N: usize = 28;

pub struct TelemetryUi {
    pub current_screen: Screen,
    touch_was_down: bool,
    last_touch_time: Instant,
    pub page_enter_time: Instant,
    trail_x: [f64; TRAIL_N],
    trail_y: [f64; TRAIL_N],
    trail_len: usize,
    trail_head: usize,
    last_trail_time: Instant,
}

impl Default for TelemetryUi {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetryUi {
    pub fn new() -> Self {
        Self {
            current_screen: Screen::Boot,
            touch_was_down: false,
            last_touch_time: Instant::now(),
            page_enter_time: Instant::now(),
            trail_x: [0.0; TRAIL_N],
            trail_y: [0.0; TRAIL_N],
            trail_len: 0,
            trail_head: 0,
            last_trail_time: Instant::now(),
        }
    }

    pub fn set_screen(&mut self, s: Screen) {
        if self.current_screen != s {
            self.current_screen = s;
            self.page_enter_time = Instant::now();
        }
    }

    pub fn cycle_screen(&mut self) {
        let idx = self.page_index();
        self.to_page((idx + 1) % PAGE_COUNT);
    }

    pub fn to_page(&mut self, i: usize) {
        self.current_screen = PAGES[i % PAGE_COUNT];
        self.page_enter_time = Instant::now();
    }

    pub fn page_index(&self) -> usize {
        for (i, p) in PAGES.iter().enumerate() {
            if *p == self.current_screen {
                return i;
            }
        }
        0
    }

    fn update_trail(&mut self, x: f64, y: f64) {
        let now = Instant::now();
        if now.duration_since(self.last_trail_time).as_millis() < 200 {
            return;
        }
        self.last_trail_time = now;
        if self.trail_len > 0 {
            let last = (self.trail_head + TRAIL_N - 1) % TRAIL_N;
            let d = (x - self.trail_x[last]).hypot(y - self.trail_y[last]);
            if d < 1.0 {
                return;
            }
        }
        self.trail_x[self.trail_head] = x;
        self.trail_y[self.trail_head] = y;
        self.trail_head = (self.trail_head + 1) % TRAIL_N;
        if self.trail_len < TRAIL_N {
            self.trail_len += 1;
        }
    }

    pub fn draw_text_s(display: &mut Display, x: i16, y: i16, col: Color, text: &str) {
        let txt = Text::from_string(
            text,
            Font::new(FontSize::SMALL, FontFamily::Monospace),
            [x, y],
        );
        display.draw_text(&txt, col, None);
    }

    pub fn draw_text_m(display: &mut Display, x: i16, y: i16, col: Color, text: &str) {
        let txt = Text::from_string(
            text,
            Font::new(FontSize::MEDIUM, FontFamily::Monospace),
            [x, y],
        );
        display.draw_text(&txt, col, None);
    }

    pub fn draw_text_c(display: &mut Display, cx: i16, y: i16, col: Color, text: &str) {
        let mut txt = Text::from_string(
            text,
            Font::new(FontSize::SMALL, FontFamily::Monospace),
            [cx, y],
        );
        txt.align(Alignment::Center, Alignment::Start);
        display.draw_text(&txt, col, None);
    }

    pub fn fill_rect(display: &mut Display, x0: i16, y0: i16, x1: i16, y1: i16, c: Color) {
        let w = (x1 - x0).max(1) as u16;
        let h = (y1 - y0).max(1) as u16;
        let r = Rect::from_dimensions([x0, y0], w, h);
        display.fill(&r, c);
    }

    pub fn outline_rect(display: &mut Display, x0: i16, y0: i16, x1: i16, y1: i16, c: Color) {
        let w = (x1 - x0).max(1) as u16;
        let h = (y1 - y0).max(1) as u16;
        let r = Rect::from_dimensions([x0, y0], w, h);
        display.stroke(&r, c);
    }

    pub fn draw_line(display: &mut Display, x0: i16, y0: i16, x1: i16, y1: i16, c: Color) {
        let l = Line::new([x0, y0], [x1, y1]);
        display.fill(&l, c);
    }

    pub fn tab_x(i: usize) -> (i16, i16) {
        let total = PAGE_COUNT as i16 * TAB_W + (PAGE_COUNT as i16 - 1) * TAB_GAP;
        let x0 = (SCR_W - total) / 2 + i as i16 * (TAB_W + TAB_GAP);
        let x1 = x0 + TAB_W;
        (x0, x1)
    }

    pub fn draw_top_bar(display: &mut Display, robot_state: &RobotState, errors: &ErrorSystem) {
        Self::draw_text_m(display, SCR_W / 2 - 25, 3, C_GOLD, "1831B");
        Self::draw_line(display, 0, TOP_H, SCR_W, TOP_H, C_GOLD_D);

        let mode = robot_state.get_mode();
        let mode_str = match mode {
            Mode::Boot => "BOOT",
            Mode::Disabled => "DISABLED",
            Mode::AutonSelect => "READY",
            Mode::Autonomous => "AUTON",
            Mode::Driver => "DRIVER",
            Mode::ErrorRecovery => "RECOVERY",
        };
        let mode_col = if mode == Mode::ErrorRecovery { C_BAD } else { C_GOOD };
        Self::fill_rect(display, 8, 5, 82, 19, C_PANEL2);
        Self::fill_rect(display, 15, 10, 21, 14, mode_col);
        Self::draw_text_s(display, 27, 8, C_WHITE, mode_str);

        let batt = vexide::battery::capacity() * 100.0;
        let bc = if batt > 50.0 { C_GOOD } else if batt > 25.0 { C_WARN } else { C_BAD };
        Self::fill_rect(display, SCR_W - 72, 5, SCR_W - 8, 19, C_PANEL2);
        Self::fill_rect(display, SCR_W - 66, 9, SCR_W - 48, 15, C_TRACK);
        let fw = ((batt / 100.0).clamp(0.0, 1.0) * 18.0) as i16;
        if fw > 0 {
            Self::fill_rect(display, SCR_W - 66, 9, SCR_W - 66 + fw, 15, bc);
        }
        let bstr = format!("{:.0}%", batt);
        Self::draw_text_s(display, SCR_W - 44, 8, C_WHITE, &bstr);
        let _ = errors;
    }

    pub fn draw_bottom_bar(&self, display: &mut Display, errors: &ErrorSystem) {
        Self::draw_line(display, 0, BOT_Y, SCR_W, BOT_Y, C_PANEL2);
        let idx = self.page_index();
        let faults = errors.active_count();

        for (i, name) in PAGE_NAMES.iter().enumerate() {
            let (x0, x1) = Self::tab_x(i);
            let on = i == idx;
            Self::fill_rect(display, x0, BOT_Y + 5, x1, BOT_Y + 31, if on { C_GOLD } else { C_PANEL });
            Self::draw_text_c(display, (x0 + x1) / 2, BOT_Y + 14, if on { C_BG } else { C_MUTED }, name);

            if PAGES[i] == Screen::Faults && faults > 0 && !on {
                let col = if errors.has_critical() { C_BAD } else { C_WARN };
                Self::fill_rect(display, x1 - 14, BOT_Y + 14, x1 - 6, BOT_Y + 22, col);
            }
        }
    }

    pub fn draw_home(
        &mut self,
        display: &mut Display,
        odom: &Odometry,
        dt: &Drivetrain,
        lift: &Lift,
        top_mech: &TopMechanism,
        auton_sel: &AutonomousSelector,
    ) {
        let size = BODY_Y1 - BODY_Y0 - 4;
        let mx = 10;
        let my = BODY_Y0 + 2;

        let pose = odom.get_pose();
        self.update_trail(pose.x, pose.y);

        let scale = size as f64 / config::FIELD_SIZE;
        let px = |fx: f64| mx + (fx * scale + 0.5) as i16;
        let py = |fy: f64| my + size - (fy * scale + 0.5) as i16;

        Self::fill_rect(display, mx - 2, my - 2, mx + size + 2, my + size + 2, C_PANEL);

        for g in 1..=5 {
            let v = g as f64 * config::FIELD_SIZE / 6.0;
            Self::draw_line(display, px(v), my + 2, px(v), my + size - 2, C_TRACK);
            Self::draw_line(display, mx + 2, py(v), mx + size - 2, py(v), C_TRACK);
        }
        Self::outline_rect(display, mx, my, mx + size, my + size, C_GOLD_D);

        for g in &field::GOALS {
            let cx = px(g.x);
            let cy = py(g.y);
            let r = 3.max((config::GOAL_RADIUS * scale + 0.5) as i16);
            let c = if g.tier == 2 { C_GOLD } else if g.tier == 1 { C_MUTED } else { C_WHITE };
            Self::outline_rect(display, cx - r, cy - r, cx + r, cy + r, c);
        }

        for i in 0..self.trail_len {
            let idx = (self.trail_head + TRAIL_N - self.trail_len + i) % TRAIL_N;
            let tc = if i * 3 > self.trail_len * 2 { C_GOLD } else { C_GOLD_D };
            Self::fill_rect(display, px(self.trail_x[idx]) - 1, py(self.trail_y[idx]) - 1, px(self.trail_x[idx]) + 1, py(self.trail_y[idx]) + 1, tc);
        }

        let hr = pose.heading * PI / 180.0;
        let sn = hr.sin();
        let cs = hr.cos();
        let hl = config::ROBOT_LENGTH / 2.0;
        let hw = config::ROBOT_WIDTH / 2.0;
        let corners = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)];
        let mut q = [(0, 0); 4];
        for (i, (lx, ly)) in corners.iter().enumerate() {
            let wx = pose.x + lx * cs + ly * sn;
            let wy = pose.y - lx * sn + ly * cs;
            q[i] = (px(wx), py(wy));
        }
        for i in 0..4 {
            let next = (i + 1) % 4;
            Self::draw_line(display, q[i].0, q[i].1, q[next].0, q[next].1, C_GOLD);
        }
        let rx = px(pose.x);
        let ry = py(pose.y);
        let nx = px(pose.x + hl * 1.5 * sn);
        let ny = py(pose.y + hl * 1.5 * cs);
        Self::draw_line(display, rx, ry, nx, ny, C_WHITE);
        Self::fill_rect(display, rx - 1, ry - 1, rx + 1, ry + 1, C_GOLD);

        let right_x = mx + size + 18;
        let tx = right_x + 140;

        Self::draw_text_s(display, tx, BODY_Y0 + 6, C_MUTED, "X");
        Self::draw_text_m(display, tx + 18, BODY_Y0 + 3, C_WHITE, &format!("{:.1}\"", pose.x));
        Self::draw_text_s(display, tx, BODY_Y0 + 28, C_MUTED, "Y");
        Self::draw_text_m(display, tx + 18, BODY_Y0 + 25, C_WHITE, &format!("{:.1}\"", pose.y));
        Self::draw_text_s(display, tx, BODY_Y0 + 50, C_MUTED, "H");
        Self::draw_text_m(display, tx + 18, BODY_Y0 + 47, C_GOLD, &format!("{:.0}°", pose.heading));

        let chips = [
            ("IMU 1", odom.channels[0].mount_known),
            ("IMU 2", odom.channels[1].mount_known),
            ("Drive L", dt.is_ok().0),
            ("Drive R", dt.is_ok().1),
            ("Lift L", lift.is_ok().0),
            ("Lift R", lift.is_ok().1),
            ("Claw", top_mech.is_ok()),
        ];
        let cy = BODY_Y0 + 74;
        let cw = 80;
        for (i, (label, ok)) in chips.iter().enumerate() {
            let col = i % 3;
            let row = i / 3;
            let ch_x = right_x + col as i16 * cw;
            let ch_y = cy + row as i16 * 14;
            let dot_col = if *ok { C_GOOD } else { C_BAD };
            Self::fill_rect(display, ch_x + 2, ch_y + 4, ch_x + 6, ch_y + 8, dot_col);
            Self::draw_text_s(display, ch_x + 10, ch_y + 1, if *ok { C_WHITE } else { C_MUTED }, label);
        }

        let ay = BODY_Y1 - 30;
        Self::fill_rect(display, right_x, ay, SCR_W - 10, BODY_Y1, C_PANEL);
        let conf = auton_sel.is_confirmed();
        Self::draw_text_s(display, right_x + 10, ay + 5, C_MUTED, "AUTON");
        Self::draw_text_s(display, right_x + 10, ay + 17, if conf { C_GOOD } else { C_GOLD }, auton_sel.get_selected_name());
        Self::fill_rect(display, SCR_W - 86, ay + 6, SCR_W - 16, ay + 24, if conf { C_GOOD } else { C_PANEL2 });
        Self::draw_text_s(display, SCR_W - 80, ay + 10, if conf { C_BG } else { C_WARN }, if conf { "LOCKED" } else { "NOT SET" });
    }

    pub fn draw_systems(
        display: &mut Display,
        dt: &Drivetrain,
        lift: &Lift,
        top_mech: &TopMechanism,
        odom: &Odometry,
    ) {
        let y0 = BODY_Y0;

        let temps = [
            ("DRV L", dt.left_temp()),
            ("DRV R", dt.right_temp()),
            ("LFT L", lift.left_temp()),
            ("LFT R", lift.right_temp()),
            ("CLAW", top_mech.claw_temp()),
        ];
        let cw = (SCR_W - 20) / 5;
        for (i, (name, t)) in temps.iter().enumerate() {
            let cx = 10 + cw * i as i16;
            let col = if *t < 40.0 { C_GOOD } else if *t < 55.0 { C_WARN } else { C_BAD };
            Self::fill_rect(display, cx + 4, y0 + 6, cx + cw - 4, y0 + 50, C_PANEL);
            Self::draw_text_m(display, cx + cw / 2 - 14, y0 + 14, col, &format!("{:.0}", t));
            Self::draw_text_c(display, cx + cw / 2, y0 + 36, C_MUTED, name);
        }

        let st = odom.get_status();
        let batt_v = vexide::battery::voltage();
        let claw_str = match top_mech.claw_state {
            crate::subsystems::top_mechanism::ClawState::Closed => "CLOSED",
            _ => "OPEN",
        };

        let tiles = [
            ("VOLTS", format!("{:.2}V", batt_v), if batt_v > 11.5 { C_GOOD } else { C_WARN }),
            ("LIFT DEG", format!("{:.0}°", lift.position()), C_WHITE),
            ("CLAW", claw_str.to_string(), if claw_str == "CLOSED" { C_GOOD } else { C_WARN }),
            ("HDG SRC", if st.heading_from_imu { "IMU" } else { "ENC" }.to_string(), C_GOOD),
            ("WALL FIX", if st.anchored { "ON" } else { "OFF" }.to_string(), if st.anchored { C_GOOD } else { C_MUTED }),
            ("SPEED", format!("{:.1}\"/s", st.speed), C_WHITE),
            ("TURN RATE", format!("{:.0}°/s", st.turn_rate), C_WHITE),
            ("TURN SCL", format!("{:.4}", odom.turn_scale), C_GOLD),
        ];

        let ty = y0 + 70;
        let tw = (SCR_W - 20 - 3 * 6) / 4;
        let th = 42;
        for (i, (label, val, col)) in tiles.iter().enumerate() {
            let x = 10 + (i as i16 % 4) * (tw + 6);
            let y = ty + (i as i16 / 4) * (th + 8);
            Self::fill_rect(display, x, y, x + tw, y + th, C_PANEL);
            Self::draw_text_m(display, x + 8, y + 6, *col, val);
            Self::draw_text_s(display, x + 8, y + 26, C_MUTED, label);
        }
    }

    pub fn draw_auton(&self, display: &mut Display, auton_sel: &AutonomousSelector) {
        let total = auton_sel.count();
        if total == 0 {
            Self::draw_text_c(display, SCR_W / 2, 100, C_BAD, "No autonomous routines");
            return;
        }

        let sel = auton_sel.get_selection();
        let conf = auton_sel.is_confirmed();
        let on_col = if conf { C_GOOD } else { C_GOLD };

        let cols = 2;
        let per_col = (total + cols - 1) / cols;
        let rh = 30;
        let w = (SCR_W - 28) / cols as i16;

        for i in 0..total {
            let col = (i / per_col) as i16;
            let r = (i % per_col) as i16;
            let x0 = 10 + col * (w + 8);
            let y0 = BODY_Y0 + r * (rh + 4);
            let x1 = x0 + w;
            let y1 = y0 + rh;
            let on = i == sel;

            Self::fill_rect(display, x0, y0, x1, y1, if on { on_col } else { C_PANEL });
            let num = format!("{}.", i + 1);
            Self::draw_text_s(display, x0 + 8, y0 + 8, if on { C_BG } else { C_MUTED }, &num);
            Self::draw_text_s(display, x0 + 26, y0 + 8, if on { C_BG } else { C_WHITE }, auton_sel.name_at(i));
        }

        let dy = BODY_Y1 - 36;
        Self::fill_rect(display, 10, dy, SCR_W - 10, BODY_Y1, C_PANEL);
        Self::draw_text_s(display, 22, dy + 6, on_col, auton_sel.name_at(sel));
        Self::draw_text_s(display, 22, dy + 21, C_MUTED, auton_sel.description_at(sel));

        Self::fill_rect(display, SCR_W - 130, dy + 5, SCR_W - 16, dy + 31, if conf { C_GOOD } else { C_GOLD });
        Self::draw_text_c(display, SCR_W - 73, dy + 12, C_BG, if conf { "LOCKED IN" } else { "CONFIRM" });
    }

    pub fn draw_faults(&self, display: &mut Display, errors: &ErrorSystem) {
        let mut row_y = BODY_Y0;
        let mut shown = 0;

        for code in &ErrorCode::ALL {
            let e = errors.get(*code);
            if !e.active {
                continue;
            }
            let crit = e.severity == Severity::Critical;
            let col = if crit { C_BAD } else { C_WARN };

            Self::fill_rect(display, 10, row_y, SCR_W - 10, row_y + 26, C_PANEL);
            Self::fill_rect(display, 18, row_y + 10, 24, row_y + 16, col);
            Self::draw_text_s(display, 32, row_y + 8, C_WHITE, code.name());
            Self::draw_text_s(display, SCR_W - 100, row_y + 8, col, if crit { "CRITICAL" } else { "WARNING" });

            row_y += 30;
            shown += 1;
            if row_y > BODY_Y1 - 28 {
                break;
            }
        }

        if shown == 0 {
            Self::draw_text_c(display, SCR_W / 2, 90, C_GOOD, "ALL CLEAR");
            Self::draw_text_c(display, SCR_W / 2, 115, C_MUTED, "No active faults :)");
        }
    }

    pub fn handle_touch(
        &mut self,
        display: &mut Display,
        auton_sel: &mut AutonomousSelector,
    ) {
        let touch = display.touch_status();
        let down = touch.state == TouchState::Pressed || touch.state == TouchState::Held;
        let fresh = down && !self.touch_was_down;
        self.touch_was_down = down;

        let now = Instant::now();
        if !fresh || self.current_screen == Screen::Boot || now.duration_since(self.last_touch_time).as_millis() < 200 {
            return;
        }
        self.last_touch_time = now;

        let tx = touch.point.x;
        let ty = touch.point.y;

        if ty >= BOT_Y {
            for i in 0..PAGE_COUNT {
                let (x0, x1) = Self::tab_x(i);
                if tx >= x0 - TAB_GAP / 2 && tx <= x1 + TAB_GAP / 2 {
                    self.to_page(i);
                    return;
                }
            }
            return;
        }

        if ty < TOP_H {
            self.cycle_screen();
            return;
        }

        if self.current_screen == Screen::AutonSelect && auton_sel.count() > 0 {
            let total = auton_sel.count();
            let dy = BODY_Y1 - 36;
            if ty >= dy {
                if tx >= SCR_W - 130 {
                    if auton_sel.is_confirmed() {
                        auton_sel.unconfirm();
                    } else {
                        auton_sel.confirm();
                    }
                }
                return;
            }

            let cols = 2;
            let per_col = (total + cols - 1) / cols;
            let rh = 30;
            let w = (SCR_W - 28) / cols as i16;
            for i in 0..total {
                let col = (i / per_col) as i16;
                let r = (i % per_col) as i16;
                let x0 = 10 + col * (w + 8);
                let y0 = BODY_Y0 + r * (rh + 4);
                let x1 = x0 + w;
                let y1 = y0 + rh;
                if tx >= x0 && tx <= x1 && ty >= y0 && ty <= y1 {
                    auton_sel.select(i as isize);
                    return;
                }
            }
            return;
        }

        self.cycle_screen();
    }

    pub fn render(
        &mut self,
        display: &mut Display,
        robot_state: &RobotState,
        errors: &ErrorSystem,
        odom: &Odometry,
        dt: &Drivetrain,
        lift: &Lift,
        top_mech: &TopMechanism,
        auton_sel: &mut AutonomousSelector,
    ) {
        if self.current_screen == Screen::Boot {
            return;
        }

        self.handle_touch(display, auton_sel);

        display.erase(C_BG);
        match self.current_screen {
            Screen::Home => self.draw_home(display, odom, dt, lift, top_mech, auton_sel),
            Screen::Systems => Self::draw_systems(display, dt, lift, top_mech, odom),
            Screen::AutonSelect => self.draw_auton(display, auton_sel),
            Screen::Faults => self.draw_faults(display, errors),
            Screen::Boot => {}
        }
        Self::draw_top_bar(display, robot_state, errors);
        self.draw_bottom_bar(display, errors);
        display.render();
    }

    pub fn show_boot_phase(display: &mut Display, phase_name: &str, waiting: bool) {
        display.erase(C_BG);
        Self::draw_text_m(display, SCR_W / 2 - 25, 72, C_GOLD, "1831B");
        Self::draw_text_s(display, SCR_W / 2 - 50, 100, C_WHITE, "OVERRIDE 26-27");
        let sub = format!("{} {}", phase_name, if waiting { "..." } else { "OK" });
        Self::draw_text_c(display, SCR_W / 2, 140, C_MUTED, &sub);
        display.render();
    }

    pub fn show_boot_complete(display: &mut Display, all_ok: bool) {
        display.erase(C_BG);
        Self::draw_text_m(display, SCR_W / 2 - 25, 72, C_GOLD, "1831B");
        Self::draw_text_s(display, SCR_W / 2 - 50, 100, C_WHITE, "OVERRIDE 26-27");
        Self::draw_text_c(display, SCR_W / 2, 140, if all_ok { C_GOOD } else { C_WARN }, if all_ok { "ALL SYSTEMS GO" } else { "CHECK FAULTS PAGE" });
        display.render();
    }
}
