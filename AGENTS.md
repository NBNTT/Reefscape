# Reefscape Drivetrain — Agent Rules

FRC team **8567** drivetrain code. Java 17, WPILib 2025, command-based.
Branch: `main`. See `docs/BUILD.md` and `docs/VALIDATION.md`.

- `origin` → `https://github.com/NBNTT/Reefscape.git` (your fork)
- `upstream` → `https://github.com/ultraviolet8567/Reefscape.git` (team)

## Commands

```bash
./gradlew build                 # compile + spotless + test
./gradlew test                  # simulation and logic tests
./gradlew simulateJava          # desktop sim (run on the host, not the devbox)
./gradlew deploy                # roboRIO, disabled robot only
./gradlew spotlessApply         # fix formatting
```

## Stack

- GradleRIO **2025.1.1**, Java 17.
- AdvantageKit **4.1.2** (required for `linuxarm64`), Phoenix6 **25.2.0**,
  REVLib **2025.0.1**. The `akit-autolog` processor in `build.gradle` must match
  the vendor JSON version.
- No PathPlanner in this season.
- Source: `subsystems/drivetrain/Swerve`, `SwerveModule`, `PoseHistory`,
  `SwerveModuleIO*`, `GyroIO*`; `commands/drivetrain/SwerveTeleOp`, `Autos`,
  `ResetHeading`; `DriverControls`, `Constants`, `RobotContainer`. Add future
  superstructure/vision beside them, not inside the drivetrain.
- Hardware: `Constants.kHardware = REV_MK4N` is the 2024 test robot;
  `CTRE_MK5N` is the future Kraken robot, needs real IDs/offsets filled, and is
  gated by `Constants.kCtreConfigured = false`.
- Drive control is measured wheel velocity with feedforward; do not reintroduce
  open-loop percent output or SparkMax onboard velocity control.
- Zero chassis speeds hold the current wheel angles. Do not pass WPILib's
  zero-speed module headings straight to the modules; they snap the wheels.
- Non-finite stick, speed, pose, or module-state inputs must be treated as zero
  or rejected before they reach a motor.
- Stick input uses deadband rescaling plus a squared response; do not re-add the
  old `1 / 0.8` or `1 / 0.9` raw boost. Full stick must still reach the
  configured maximum.
- Heading hold uses the separate `HeadingConstants` PID. Manual rotation must
  override it; it must stay off in robot-oriented mode, X-lock, disabled,
  autonomous, and whenever the gyro is unhealthy or has no valid reading.
- Start and Back reset the gyro and pose heading together in Disabled, Teleop,
  and Test. They are intentionally not bound while an autonomous is running.
  Back was the original 2025 binding; do not remove either without an explicit
  control decision.
- Physical controls live in `DriverControls`. Drivetrain code takes semantic
  suppliers and must not import a controller class.
- Vision must go through `Swerve.addVisionMeasurement` from the robot main
  loop only, so age, order, covariance, field bounds, historical-pose,
  innovation, and accepted/rejected logs are preserved. The estimator is not
  thread-safe.
- A module with a bad encoder reading must never steer or drive. A sustained
  module fault stops the whole drivetrain; do not add three-wheel driving
  without validated control allocation and tests.
- A gyro fault must force robot-oriented driving and report the fallback; do not
  keep field-oriented control on a stale heading.

## Read first

- `Constants.java` before changing any number, ID, or limit.
- `RobotContainer.java` before changing wiring or bindings.
- The subsystem and command you are touching.
- This is the **2025 API**. Do not copy 2026 vendor calls from Rebuilt/Biocore
  without checking this repo's `vendordeps/`.

## Rules

- Subsystems own hardware; commands own behavior.
- CAN IDs and ports live in `Constants.CAN`. Never change one without explicit
  approval — it is wiring, not code.
- Units are meters, radians, seconds. Convert at the boundary with `Units`.
- Log telemetry with AdvantageKit `Logger.recordOutput`, not `System.out`.
- Every mechanism needs a simulation path (an IO sim or `simulationPeriodic()`).
- Alliance logic must use `FieldConstants` / `AllianceFlipUtil` when game code
  is added. Never hardcode Red/Blue. With no Driver Station, default to Blue.
- Declare `addRequirements(...)` correctly and clean up in `end(interrupted)`.
- No blocking calls in periodic methods. Do not call the scheduler outside
  `Robot.robotPeriodic()`.
- Run `./gradlew spotlessApply` after edits.
- `XboxController.getRightBumper()` is deprecated in this season; use the
  current trigger API when touching that binding.

## Verification

- `SwerveTest` covers simulation driving and X-lock. `SwerveControlTest` covers
  module order/kinematics, gyro sign, Start heading reset, angle wrapping, pose
  resets, steering optimization, cosine compensation, stop/disabled/
  initialization behavior, teleop interruption, field- and robot-oriented
  driving at multiple headings, heading-hold capture/hold/manual override/
  wraparound/reset, periodic drive-encoder stability, drive-voltage limits,
  vendor steering, measured velocity control, the autonomous foundation, vision
  gating and delayed history, encoder/gyro fault and recovery, module-fault safe
  stop, semantic mode transitions, disabled reset, startup validation, CTRE
  steering wrap, and CTRE config retry. `ConstantsTest` covers wiring validation.
  `PoseHistoryTest` covers the delayed-vision history ring. `SwerveTeleOpTest`
  covers input shaping. Expand them before changing behavior.
- `./gradlew build` proves compile + formatting + all tests.
- `simulateJava` on the host is the fastest behavior check; only the robot
  proves hardware.
- Never claim a deploy succeeded without checking the RioLog.