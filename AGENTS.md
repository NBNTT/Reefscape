# Reefscape 2025 — Agent Rules

FRC team **8567** robot code. Java 17, WPILib 2025, command-based.
Branch: `main`.

- `origin` → `https://github.com/NBNTT/Reefscape.git` (your fork)
- `upstream` → `https://github.com/ultraviolet8567/Reefscape.git` (team)

## Commands

```bash
./gradlew build                 # compile + spotless + test
./gradlew test                  # simulation smoke test
./gradlew simulateJava          # desktop sim (run on the host, not the devbox)
./gradlew deploy                # roboRIO, disabled robot only
./gradlew spotlessApply         # fix formatting
```

## Stack

- GradleRIO **2025.1.1**, Java 17.
- AdvantageKit, Phoenix6 **25.2.0**, REVLib **2025.0.1**.
- No PathPlanner in this season.
- Source: `Swerve`, `SwerveModule`, `SwerveModuleIO*`, `GyroIO*`,
  `SwerveTeleOp`, `Constants`, `RobotContainer`.
- Hardware: `Constants.kHardware = REV_MK4N` is the 2024 test robot;
  `CTRE_MK5N` is the future Kraken robot and needs real IDs/offsets filled.

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
- Every mechanism needs a simulation path and a `simulationPeriodic()`.
- Alliance logic must use `FieldConstants` / `AllianceFlipUtil` when game code
  is added. Never hardcode Red/Blue. With no Driver Station, default to Blue.
- Declare `addRequirements(...)` correctly and clean up in `end(interrupted)`.
- No blocking calls in periodic methods. Do not call the scheduler outside
  `Robot.robotPeriodic()`.
- Run `./gradlew spotlessApply` after edits.
- `XboxController.getRightBumper()` is deprecated in this season; use the
  current trigger API when touching that binding.

## Verification

- There is a simulation smoke test suite (`SwerveTest`). Expand it before
  changing behavior (or get explicit approval to change untested code).
- `./gradlew build` proves compile + formatting + the simulation test.
- `simulateJava` on the host is the fastest behavior check; only the robot
  proves hardware.
- Never claim a deploy succeeded without checking the RioLog.