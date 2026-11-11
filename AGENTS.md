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
- Source: `Swerve`, `SwerveModule`, `SwerveModuleIO*`, `GyroIO*`,
  `SwerveTeleOp`, `Autos`, `Constants`, `RobotContainer`.
- Hardware: `Constants.kHardware = REV_MK4N` is the 2024 test robot;
  `CTRE_MK5N` is the future Kraken robot and needs real IDs/offsets filled.
- Drive control is measured wheel velocity with feedforward; do not reintroduce
  open-loop percent output.
- Vision must go through `Swerve.addVisionMeasurement` so age, order,
  covariance, jump gates, and accepted/rejected logs are preserved.

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
  module order/kinematics, gyro sign, angle wrapping, pose resets, steering
  optimization, cosine compensation, stop/disabled/initialization behavior,
  teleop interruption, field- and robot-oriented driving at multiple headings,
  periodic drive-encoder stability, drive-voltage limits, vendor steering,
  measured velocity control, the autonomous foundation, and vision gating.
  Expand them before changing behavior.
- `./gradlew build` proves compile + formatting + all tests.
- `simulateJava` on the host is the fastest behavior check; only the robot
  proves hardware.
- Never claim a deploy succeeded without checking the RioLog.