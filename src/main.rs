//! VEX V5 Robotics Competition 2026-2027 entrypoint for Team 1831B.

use vexide::prelude::*;

use robot_1831b::config;
use robot_1831b::control::auton_selector::AutonomousSelector;
use robot_1831b::control::autonomous::run_route_by_index;
use robot_1831b::control::opcontrol::DriverControl;
use robot_1831b::error_system::ErrorSystem;
use robot_1831b::localization::mcl::Mcl;
use robot_1831b::robot_state::{BootPhase, Mode, RobotState};
use robot_1831b::subsystems::drivetrain::Drivetrain;
use robot_1831b::subsystems::lift::Lift;
use robot_1831b::subsystems::odometry::{Odometry, OdometryContext, Sensors};
use robot_1831b::subsystems::top_mechanism::TopMechanism;
use robot_1831b::ui::telemetry::{Screen, TelemetryUi};

struct Robot {
    drivetrain: Drivetrain,
    lift: Lift,
    top_mech: TopMechanism,
    imu: InertialSensor,
    imu2: InertialSensor,
    forward_tracker: RotationSensor,
    dist_left: DistanceSensor,
    dist_right: DistanceSensor,
    dist_front: DistanceSensor,
    controller: Controller,
    display: Display,

    odometry: Odometry,
    mcl: Mcl,
    auton_selector: AutonomousSelector,
    error_system: ErrorSystem,
    robot_state: RobotState,
    telemetry_ui: TelemetryUi,
    driver_control: DriverControl,
}

impl Robot {
    pub async fn initialize(peripherals: Peripherals) -> Self {
        let mut display = peripherals.display;
        let controller = peripherals.primary_controller;
        let mut error_system = ErrorSystem::new();
        let robot_state = RobotState::new();

        robot_state.set_mode(Mode::Boot);

        // Phase 1: Hardware Init
        robot_state.set_boot_phase(BootPhase::HardwareInit);
        TelemetryUi::show_boot_phase(&mut display, BootPhase::HardwareInit.name(), true);

        let left_front = Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward);
        let left_mid = Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward);
        let left_back = Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward);

        let right_front = Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse);
        let right_mid = Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse);
        let right_back = Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse);

        let drivetrain = Drivetrain::new(
            left_front, left_mid, left_back,
            right_front, right_mid, right_back,
        );

        let lift_l_dir = if config::LIFT_LEFT_REVERSED { Direction::Reverse } else { Direction::Forward };
        let lift_r_dir = if config::LIFT_RIGHT_REVERSED { Direction::Reverse } else { Direction::Forward };
        let lift_left = Motor::new(peripherals.port_9, Gearset::Green, lift_l_dir);
        let lift_right = Motor::new(peripherals.port_10, Gearset::Green, lift_r_dir);
        let lift = Lift::new(lift_left, lift_right);

        let claw_motor = Motor::new(peripherals.port_16, Gearset::Green, Direction::Forward);
        let top_mech = TopMechanism::new(claw_motor);

        let mut imu = InertialSensor::new(peripherals.port_13);
        let mut imu2 = InertialSensor::new(peripherals.port_14);
        let forward_tracker = RotationSensor::new(peripherals.port_18, Direction::Forward);

        let dist_left = DistanceSensor::new(peripherals.port_7);
        let dist_right = DistanceSensor::new(peripherals.port_8);
        let dist_front = DistanceSensor::new(peripherals.port_15);

        TelemetryUi::show_boot_phase(&mut display, BootPhase::HardwareInit.name(), false);
        sleep(core::time::Duration::from_millis(200)).await;

        // Phase 2: IMU Calibrate
        robot_state.set_boot_phase(BootPhase::ImuCalibrate);
        TelemetryUi::show_boot_phase(&mut display, BootPhase::ImuCalibrate.name(), true);

        let _ = imu.calibrate().await;
        let _ = imu2.calibrate().await;

        TelemetryUi::show_boot_phase(&mut display, BootPhase::ImuCalibrate.name(), false);
        sleep(core::time::Duration::from_millis(200)).await;

        // Phase 3: Sensor Check
        robot_state.set_boot_phase(BootPhase::SensorCheck);
        TelemetryUi::show_boot_phase(&mut display, BootPhase::SensorCheck.name(), true);

        let imu1_ok = imu.rotation().is_ok();
        let imu2_ok = imu2.rotation().is_ok();
        let trk_ok = forward_tracker.position().is_ok();
        let batt_v = vexide::battery::voltage();
        let max_temp = drivetrain.left_temp().max(drivetrain.right_temp());
        error_system.update_hardware(imu1_ok, imu2_ok, trk_ok, batt_v, max_temp);

        sleep(core::time::Duration::from_millis(200)).await;

        // Phase 4: Subsystem Init
        robot_state.set_boot_phase(BootPhase::SubsystemInit);
        TelemetryUi::show_boot_phase(&mut display, BootPhase::SubsystemInit.name(), true);

        let odometry = Odometry::new();
        let mcl = Mcl::new(0.0, 0.0, 0.0);

        sleep(core::time::Duration::from_millis(200)).await;

        // Phase 5: Auton Setup
        robot_state.set_boot_phase(BootPhase::AutonSetup);
        TelemetryUi::show_boot_phase(&mut display, BootPhase::AutonSetup.name(), true);

        let mut auton_selector = AutonomousSelector::new();
        auton_selector.add_routine("Left Side", "Mid-wall, score 2");
        auton_selector.add_routine("Right Side", "Mid-wall, score 2");
        auton_selector.add_routine("Left Mirror", "Left side, flipped top to bottom");
        auton_selector.add_routine("Right Mirror", "Right side, flipped top to bottom");
        auton_selector.add_routine("Safe Mirror", "Preload only, flipped");
        auton_selector.add_routine("Safe", "Preload only");
        auton_selector.add_routine("Spin Calibrate", "Spin 2 turns, set turn scale");
        auton_selector.add_routine("Skills", "Match path + 3rd pin");

        sleep(core::time::Duration::from_millis(200)).await;

        // Phase 6: Ready
        robot_state.set_boot_phase(BootPhase::Ready);
        let all_ok = !error_system.has_critical();
        TelemetryUi::show_boot_complete(&mut display, all_ok);
        sleep(core::time::Duration::from_millis(1500)).await;

        robot_state.set_mode(Mode::AutonSelect);
        let mut telemetry_ui = TelemetryUi::new();
        telemetry_ui.set_screen(Screen::AutonSelect);

        Self {
            drivetrain,
            lift,
            top_mech,
            imu,
            imu2,
            forward_tracker,
            dist_left,
            dist_right,
            dist_front,
            controller,
            display,
            odometry,
            mcl,
            auton_selector,
            error_system,
            robot_state,
            telemetry_ui,
            driver_control: DriverControl::new(),
        }
    }

    pub fn step_sensors_and_odometry(&mut self) {
        let sensors = Sensors::new(
            &self.imu,
            &self.imu2,
            &self.forward_tracker,
            &self.dist_left,
            &self.dist_right,
            &self.dist_front,
        );
        sensors.step_odometry(
            &self.drivetrain,
            &mut self.odometry,
            &mut self.mcl,
            &mut self.error_system,
        );
    }
}

impl Compete for Robot {
    async fn disabled(&mut self) {
        self.robot_state.set_mode(Mode::Disabled);
        self.telemetry_ui.set_screen(Screen::AutonSelect);
        self.drivetrain.coast();

        while vexide::competition::mode() == vexide::competition::CompetitionMode::Disabled {
            self.step_sensors_and_odometry();

            // Handle controller buttons for auton selector (Left, Right, A)
            if let Ok(state) = self.controller.state() {
                self.auton_selector.update_from_controller(
                    state.button_left.is_now_pressed(),
                    state.button_right.is_now_pressed(),
                    state.button_a.is_now_pressed(),
                );
            }

            self.telemetry_ui.render(
                &mut self.display,
                &self.robot_state,
                &self.error_system,
                &self.odometry,
                &self.drivetrain,
                &self.lift,
                &self.top_mech,
                &mut self.auton_selector,
            );

            sleep(core::time::Duration::from_millis(20)).await;
        }
    }

    async fn autonomous(&mut self) {
        self.robot_state.set_mode(Mode::Autonomous);
        self.telemetry_ui.set_screen(Screen::Home);
        self.error_system.clear_all();

        let sel = self.auton_selector.get_selection();
        let sensors = Sensors::new(
            &self.imu,
            &self.imu2,
            &self.forward_tracker,
            &self.dist_left,
            &self.dist_right,
            &self.dist_front,
        );
        let mut ctx = OdometryContext::new(
            &mut self.odometry,
            &mut self.mcl,
            &mut self.error_system,
            &sensors,
        );
        run_route_by_index(
            sel,
            &mut self.drivetrain,
            &mut ctx,
            &mut self.lift,
            &mut self.top_mech,
        ).await;
    }

    async fn driver(&mut self) {
        self.robot_state.set_mode(Mode::Driver);
        self.telemetry_ui.set_screen(Screen::Home);
        self.error_system.clear_all();
        self.driver_control.reset_match_timer();

        while vexide::competition::mode() == vexide::competition::CompetitionMode::Driver {
            self.step_sensors_and_odometry();

            self.driver_control.update_step(
                &mut self.controller,
                &mut self.drivetrain,
                &mut self.lift,
                &mut self.top_mech,
                &self.odometry,
                &mut self.error_system,
            ).await;

            self.telemetry_ui.render(
                &mut self.display,
                &self.robot_state,
                &self.error_system,
                &self.odometry,
                &self.drivetrain,
                &self.lift,
                &self.top_mech,
                &mut self.auton_selector,
            );

            sleep(core::time::Duration::from_millis(10)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::initialize(peripherals).await;
    robot.compete().await;
}
