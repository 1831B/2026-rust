//! Diagnostics and fault tracking system.

use std::time::Instant;
use crate::config;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    ImuMissing = 0,
    ForwardTrackerMissing,
    LeftDriveFault,
    RightDriveFault,
    LiftFault,
    ClawFlipFault,
    IntakeFault,
    ClawFault,
    LowBattery,
    MotorOverheat,
}

pub const ERROR_COUNT: usize = 10;

impl ErrorCode {
    pub const ALL: [ErrorCode; ERROR_COUNT] = [
        ErrorCode::ImuMissing,
        ErrorCode::ForwardTrackerMissing,
        ErrorCode::LeftDriveFault,
        ErrorCode::RightDriveFault,
        ErrorCode::LiftFault,
        ErrorCode::ClawFlipFault,
        ErrorCode::IntakeFault,
        ErrorCode::ClawFault,
        ErrorCode::LowBattery,
        ErrorCode::MotorOverheat,
    ];

    pub const fn name(&self) -> &'static str {
        match self {
            Self::ImuMissing => "IMU Missing",
            Self::ForwardTrackerMissing => "Fwd Tracker Miss",
            Self::LeftDriveFault => "Left Drive Fault",
            Self::RightDriveFault => "Right Drive Fault",
            Self::LiftFault => "Lift Fault",
            Self::ClawFlipFault => "Claw Flip Fault",
            Self::IntakeFault => "Intake Fault",
            Self::ClawFault => "Claw Fault",
            Self::LowBattery => "Low Battery",
            Self::MotorOverheat => "Motor Overheat",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Warning,
    Critical,
}

#[derive(Debug, Clone)]
pub struct Error {
    pub active: bool,
    pub severity: Severity,
    pub timestamp: Instant,
    pub count: u32,
    pub message: String,
}

impl Default for Error {
    fn default() -> Self {
        Self {
            active: false,
            severity: Severity::Warning,
            timestamp: Instant::now(),
            count: 0,
            message: String::new(),
        }
    }
}

#[derive(Debug)]
pub struct ErrorSystem {
    errors: [Error; ERROR_COUNT],
}

impl Default for ErrorSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl ErrorSystem {
    pub fn new() -> Self {
        Self {
            errors: std::array::from_fn(|_| Error::default()),
        }
    }

    pub fn set(&mut self, code: ErrorCode, sev: Severity, msg: &str) {
        let idx = code as usize;
        let entry = &mut self.errors[idx];
        if !entry.active {
            entry.active = true;
            entry.severity = sev;
            entry.timestamp = Instant::now();
        }
        entry.count += 1;
        entry.message.clear();
        entry.message.push_str(msg);
    }

    pub fn clear(&mut self, code: ErrorCode) {
        let idx = code as usize;
        self.errors[idx].active = false;
        self.errors[idx].count = 0;
        self.errors[idx].message.clear();
    }

    pub fn clear_all(&mut self) {
        for e in &mut self.errors {
            *e = Error::default();
        }
    }

    pub fn update_hardware(
        &mut self,
        imu_ok: bool,
        tracker_ok: bool,
        battery_volts: f64,
        max_temp: f64,
    ) {
        if !imu_ok {
            self.set(
                ErrorCode::ImuMissing,
                Severity::Critical,
                "IMU disconnected",
            );
        } else {
            self.clear(ErrorCode::ImuMissing);
        }

        if !tracker_ok {
            self.set(
                ErrorCode::ForwardTrackerMissing,
                Severity::Critical,
                "Fwd tracker lost",
            );
        } else {
            self.clear(ErrorCode::ForwardTrackerMissing);
        }

        if battery_volts < config::LOW_BATTERY_THRESHOLD {
            self.set(
                ErrorCode::LowBattery,
                Severity::Critical,
                "Battery critical",
            );
        } else {
            self.clear(ErrorCode::LowBattery);
        }

        if max_temp > 55.0 {
            self.set(
                ErrorCode::MotorOverheat,
                Severity::Warning,
                "Motor overheating",
            );
        } else {
            self.clear(ErrorCode::MotorOverheat);
        }

        // Timeout stale warnings
        let now = Instant::now();
        for entry in &mut self.errors {
            if entry.active
                && entry.severity == Severity::Warning
                && now.duration_since(entry.timestamp) > config::ERROR_TIMEOUT
            {
                entry.active = false;
            }
        }
    }

    pub fn has_critical(&self) -> bool {
        self.errors
            .iter()
            .any(|e| e.active && e.severity == Severity::Critical)
    }

    pub fn active_count(&self) -> usize {
        self.errors.iter().filter(|e| e.active).count()
    }

    pub fn get(&self, code: ErrorCode) -> &Error {
        &self.errors[code as usize]
    }
}
