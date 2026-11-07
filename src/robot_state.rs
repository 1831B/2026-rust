use core::sync::atomic::{AtomicU8, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    Boot = 0,
    Disabled,
    AutonSelect,
    Autonomous,
    Driver,
    ErrorRecovery,
}

impl From<u8> for Mode {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Boot,
            1 => Self::Disabled,
            2 => Self::AutonSelect,
            3 => Self::Autonomous,
            4 => Self::Driver,
            _ => Self::ErrorRecovery,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BootPhase {
    HardwareInit = 0,
    ImuCalibrate,
    SensorCheck,
    SubsystemInit,
    AutonSetup,
    Ready,
}

impl From<u8> for BootPhase {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::HardwareInit,
            1 => Self::ImuCalibrate,
            2 => Self::SensorCheck,
            3 => Self::SubsystemInit,
            4 => Self::AutonSetup,
            _ => Self::Ready,
        }
    }
}

impl BootPhase {
    pub const fn name(&self) -> &'static str {
        match self {
            Self::HardwareInit => "Hardware Init",
            Self::ImuCalibrate => "IMU Calibrate",
            Self::SensorCheck => "Sensor Check",
            Self::SubsystemInit => "Subsystem Init",
            Self::AutonSetup => "Auton Setup",
            Self::Ready => "Ready",
        }
    }
}

#[derive(Debug, Default)]
pub struct RobotState {
    mode: AtomicU8,
    boot_phase: AtomicU8,
}

impl RobotState {
    pub const fn new() -> Self {
        Self {
            mode: AtomicU8::new(Mode::Boot as u8),
            boot_phase: AtomicU8::new(BootPhase::HardwareInit as u8),
        }
    }

    pub fn set_mode(&self, m: Mode) {
        self.mode.store(m as u8, Ordering::Release);
    }

    pub fn get_mode(&self) -> Mode {
        Mode::from(self.mode.load(Ordering::Acquire))
    }

    pub fn set_boot_phase(&self, p: BootPhase) {
        self.boot_phase.store(p as u8, Ordering::Release);
    }

    pub fn get_boot_phase(&self) -> BootPhase {
        BootPhase::from(self.boot_phase.load(Ordering::Acquire))
    }

    pub fn is_ready(&self) -> bool {
        self.get_mode() != Mode::Boot
    }
}
