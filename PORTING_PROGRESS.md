# 1831B Port: PROS C++ → vexide Rust

**Source:** [`~/projects/1831B-`](https://github.com/ng_tz/1831B-) · HEAD `0a18ff3` "The New Big code change to Rust"  
**Target:** `~/projects/2026-rust` · crate `robot-1831b` · vexide `0.8.0`  
**Toolchain:** `nightly-2025-11-26` (pinned via `rust-toolchain.toml`)  
**Build target:** `armv7a-vex-v5`

---

## Build Status

```bash
cd ~/projects/2026-rust

# Development build (passes ✅, zero warnings)
cargo v5 build

# Release build (passes ✅, zero warnings)
cargo v5 build --release

# Cross-target type-check only (passes ✅, zero warnings)
cargo +nightly-2025-11-26 check --target armv7a-vex-v5
```

**Debug binary:** `target/armv7a-vex-v5/debug/robot-1831b.bin` (~789 KB)  
**Release binary:** `target/armv7a-vex-v5/release/robot-1831b.bin` (149 KB with `opt-level = "z"` and `lto = "fat"`)

---

## File Map: C++ → Rust

| C++ source | Rust equivalent | Notes |
|---|---|---|
| `include/robot/config.hpp` | `src/config.rs` | All motor ports, PID gains, field geometry constants |
| `include/robot/field.hpp` | `src/field.rs` | Field coords, `GoalSpot`, `GOALS` array, `at()` fn |
| `include/robot/error_system.hpp` | `src/error_system.rs` | 10 error codes incl. `Imu2Missing`, `ImuMismatch` |
| `include/robot/robot_state.hpp` | `src/robot_state.rs` | `Mode`/`BootPhase` enums via `AtomicU8` |
| `src/control/pid.cpp` | `src/control/pid.rs` | PID with anti-windup, settle detection |
| `src/control/motion.cpp` | `src/control/motion.rs` | drive_distance, turn_to_heading, move_to_point, follow_path, swing_turn, approach_goal, drive_straight_to, calibrate_turn_scale |
| `src/control/ramsete.cpp` | `src/control/ramsete.rs` | RAMSETE controller, follow, make_linear, make_turn |
| `src/control/auton_selector.cpp` | `src/control/auton_selector.rs` | Touch-driven & controller auton select/confirm/unconfirm |
| `src/control/autonomous.cpp` | `src/control/autonomous.rs` | route_mid_wall + all 8 routes via `run_route_by_index` |
| `src/control/opcontrol.cpp` | `src/control/opcontrol.rs` | Arcade drive, claw toggle, lift manual+presets, e-stop, telemetry cycle |
| `src/subsystems/drivetrain.cpp` | `src/subsystems/drivetrain.rs` | 3+3 motor groups, all accessors |
| `src/subsystems/lift.cpp` | `src/subsystems/lift.rs` | Dual-motor, PID hold, `move_to_position` async |
| `src/subsystems/top_mechanism.cpp` | `src/subsystems/top_mechanism.rs` | Claw only (intake/wrist stripped in `1831B-` source) |
| `src/subsystems/odometry.cpp` | `src/subsystems/odometry.rs` | Dual-IMU fusion, mount detection, turn scale, MCL integration, `Sensors` & `OdometryContext` |
| `src/localization/mcl.cpp` | `src/localization/mcl.rs` | 200-particle filter, 3 distance sensors, low-variance resampling |
| `src/ui/telemetry.cpp` | `src/ui/telemetry.rs` | HOME/SYSTEMS/AUTON/FAULTS screens, touch nav, field map, trail, robot outline |
| `src/main.cpp` | `src/main.rs` | Robot struct, `Compete` impl, sensor reading pipeline |
| `include/robot/globals.hpp` | _(inlined)_ | Peripheral ownership moved into `Robot` struct fields |

---

## Architecture Decisions

### 1. Hardware ownership in `Robot` struct
PROS used global `extern` pointers (`pros::Motor* drive_left`). In vexide, all peripheral handles are owned values — the `Robot` struct owns every device directly (motors, sensors, controller, display). This is idiomatic and enforced by the borrow checker.

### 2. Sensor data pipeline & `OdometryContext`
- `Sensors<'a>` borrows the sensor handles (`imu`, `imu2`, `forward_tracker`, `dist_left`, `dist_right`, `dist_front`) without borrowing the entire `Robot` struct, enabling split borrows of `&mut drivetrain`, `&mut lift`, `&mut top_mech`.
- `OdometryContext<'a>` bundles `&mut Odometry`, `&mut Mcl`, `&mut ErrorSystem`, and `&Sensors`.
- Inside all motion control loops (`drive_distance`, `turn_to_heading`, `move_to_point`, `approach_goal`, `follow_path`, `swing_turn`, `calibrate_turn_scale`, and `ramsete::follow`), `ctx.step(dt)` is invoked every 10ms cycle. This ensures heading, position, tracker, and error states are continuously updated during motion, faithfully mirroring the C++ PROS background odometry RTOS task.

### 3. No `globals.hpp` equivalent
C++ shared state via `globals.hpp` → Rust shared state via typed contexts (`OdometryContext`) and `AtomicU8` for lock-free Mode/BootPhase reads.

### 4. Claw-only top mechanism
`1831B-` (HEAD `0a18ff3`) already stripped intake and wrist from `top_mechanism.cpp`. The Rust port matches this — `top_mechanism.rs` is claw-only. `1831B-CODE/` still had intake; that codebase was **not** used.

### 5. Dual-IMU fusion
Odometry fuses two IMUs (ports 13 & 14 in `config.rs`). Mount detection picks which is authoritative based on `acceleration().z` magnitude. `ImuMismatch` error fires when both read heading but differ by > threshold.

### 6. Zero `unsafe`, zero dead code
Code passes `RUSTFLAGS="-D warnings"` with zero warnings across all modules.

---

## PROS → vexide API Mapping

| PROS | vexide 0.8.0 | Notes |
|---|---|---|
| `Motor::move_voltage(mV)` | `Motor::set_voltage(V)` | **Divide by 1000.** PROS uses ±12000 mV, vexide uses ±12.0 V |
| `Motor::move_absolute(deg, rpm)` | `Motor::set_position_target(Angle, rpm)` | Use `Angle::from_degrees(deg)` |
| `Motor::get_position()` | `Motor::position()` → `Result<Angle>` | `.as_degrees()` for float degrees |
| `IMU::get_heading()` | `InertialSensor::heading()` | Returns `Result<Angle>` |
| `IMU::get_rotation()` | `InertialSensor::rotation()` | Cumulative, unbounded |
| `IMU::get_accel()` | `InertialSensor::acceleration()` → `Vector3<f64>` | `.x/.y/.z` fields |
| `IMU::get_gyro_rate()` | `InertialSensor::gyro_rate()` → `Vector3<f64>` | `.x/.y/.z` fields |
| `IMU::calibrate()` / `is_calibrating()` | `InertialSensor::calibrate().await` | Async, blocks until done |
| `RotationSensor::get_position()` (centideg) | `RotationSensor::position()` → `Result<Angle>` | Multiply `.as_degrees() * 100.0` to replicate centidegrees used in odometry internals |
| `DistanceSensor::get()` (mm) | `DistanceSensor::object()` → `Result<Option<DistanceObject>>` | `.distance` is `u32` mm |
| `Controller::get_digital_new_press(BTN)` | `state.button_a.is_now_pressed()` | Poll `controller.state()` each loop |
| `Controller::get_analog(STICK)` | `state.left_stick.y_raw()` | Returns `i8` (-127..127) |
| `Controller::rumble(pattern)` | `controller.try_rumble(pattern)` | Non-blocking |
| `Controller::set_text(row, col, text)` | `controller.try_set_text(text, line, col)` | Non-blocking. **1-indexed** (lines 1..=3, cols 1..=19) |
| `pros::lcd::*` | `Display::fill(&shape, color)` / `Display::stroke(&shape, color)` | `Line` only impls `Fill`, not `Stroke` — use `fill` for lines |
| `pros::battery::get_capacity()` (%) | `vexide::battery::capacity()` | Returns 0.0–1.0, multiply by 100 for percentage |
| `pros::battery::get_voltage()` (mV) | `vexide::battery::voltage()` | Returns Volts directly |
| `task::delay(ms)` | `sleep(Duration::from_millis(ms)).await` | Requires `vexide::prelude::*` |
| `task::Task` | `vexide::task::spawn(async { ... })` | Returns `Task<T>` |
| `pros::competition::is_autonomous()` | `vexide::competition::mode()` → `CompetitionMode` | Enum match for loop termination |

---

## Status of Porting & Completed Tasks

### High priority
- [x] **Release build verified** — compiles cleanly via `cargo v5 build --release`, producing a 149 KB binary with `opt-level = "z"` and `lto = "fat"`.
- [x] **Continuous odometry updates during motion** — implemented `Sensors` and `OdometryContext` to step odometry and error system every 10ms during all autonomous moves.
- [x] **PID gain consistency verified** — PROS millivolt gains are preserved and mapped accurately to Volt outputs in `motion.rs` by dividing PID outputs by 1000.0.
- [x] **Rotation sensor tick scaling verified** — `position().as_degrees() * 100.0` correctly replicates PROS centidegree values in `odometry.rs` math (`TICKS_PER_REV = 36000.0`).

### Medium priority
- [x] **MCL dead-method cleanup** — verified zero dead methods or compiler warnings; clean pseudo-random RNG utilities.
- [x] **Controller & Touch auton selector** — dual control supported (touchscreen grid navigation + confirmation and controller Left/Right/A buttons).
- [x] **`robot_state.rs` BootPhase completeness** — confirmed all 6 variants (`HardwareInit`, `ImuCalibrate`, `SensorCheck`, `SubsystemInit`, `AutonSetup`, `Ready`) have correct names and `From<u8>` roundtrip.

### Low priority / On-Robot Validation
- [x] **Field coordinate validation** — verified `field.rs` coordinates and goal spots exactly match `1831B-/include/robot/field.hpp`.
- [x] **Git commit** — committed complete ported codebase with continuous odometry updates and verified builds.
- [x] **Binary size verification** — Release binary is 149 KB (well within Brain flash limits).
- [ ] **On-robot physical tuning** — empirical verification of PID settle thresholds and distance sensor noise model under real competition carpet conditions.

---

## Notable `1831B-` vs `1831B-CODE` Differences

This port targets `1831B-` (HEAD `0a18ff3`), **not** `1831B-CODE/`. Key differences that affected porting decisions:

| Feature | `1831B-` (this port) | `1831B-CODE/` |
|---|---|---|
| IMU count | 2 (dual-IMU fusion) | 1 |
| Top mechanism | Claw only | Intake + wrist + claw |
| Auton selector | Touch-driven screen UI + controller | Button-based |
| Field coordinates | Full field.hpp system | Minimal |
| Telemetry | 4-screen system (HOME/SYSTEMS/AUTON/FAULTS) | Basic LCD |
| Odometry | MCL-integrated | Dead-reckoning only |

---

## Quick Reference

```bash
# Build (dev)
cargo v5 build

# Build (release)
cargo v5 build --release

# Type-check only (fast)
cargo +nightly-2025-11-26 check --target armv7a-vex-v5

# Upload to robot
cargo v5 upload

# View logs (after connecting robot)
cargo v5 terminal
```

**Ports (from `src/config.rs`):**

| Device | Port |
|---|---|
| Drive Left (3 motors) | 1, 2, 3 |
| Drive Right (3 motors) | 4, 5, 6 |
| Lift (2 motors) | 9, 10 |
| Claw | 16 |
| IMU 1 | 13 |
| IMU 2 | 14 |
| Forward tracker | 18 |
| Distance sensors | Left: 7, Right: 8, Front: 15 |
