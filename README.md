# Team 8567 Drivetrain

Drivetrain-only base for the 2024 test robot and the future 2026-2027 robot.
The old robot uses REV MK4n hardware; the future robot uses CTRE MK5n with
Kraken X60/X44 motors. The same drivetrain logic runs both through a hardware
IO layer.

## Documentation

- [Build and test environment](docs/BUILD.md) — JDK, WPILib, AdvantageKit
  linuxarm64 fix, commands, deploy notes.
- [Calibration, simulation, deployment, and validation](docs/VALIDATION.md) —
  the full bench and field checklist.

## Hardware paths

The active path is selected by `Constants.kHardware`.

- `REV_MK4N`: the 2024 test robot. This is active now.
- `CTRE_MK5N`: the future Kraken robot. The code compiles, but the CTRE IDs,
  offsets, wheel size, gearing, turn rotor-to-sensor ratio, current limits, and
  steering gains are placeholders. Continuous wrap is enabled for steering.
  `Constants.kCtreConfigured` is `false` and startup refuses to create CTRE
  hardware until it is set after the values are filled in.

## Driver controls

| Control | Action |
| --- | --- |
| Left stick | Drive (Teleop or Test mode) |
| Right stick X | Turn |
| Right bumper | Slow mode, with rumble |
| A | X-lock |
| Left bumper | Toggle field-oriented / robot-oriented |
| Start | Reset current direction to field-forward (also Back) |

The controllers are **8BitDo Ultimate 2 Wireless** in Xbox mode. Start is the
`+`/Menu button and Back is the `−`/View button. The original 2025 code used
Back for the gyro reset; Start was commented out. Start is now the primary reset
as requested, and Back remains bound so the old control is not lost. Both work
while **Disabled**, **Teleop**, and **Test**. They are intentionally not bound while
an autonomous is running, where a mid-auto heading reset would corrupt the auto
pose.

All physical mapping lives in `DriverControls`; the drivetrain command receives
named suppliers and does not import a controller class. The 8BitDo extra back
buttons are not mapped yet. Move the robot-oriented toggle to one of them by
changing `DriverControls.toggleRobotOriented()`.

## Fault behavior

- Required `SwerveModuleIO` and `GyroIO` methods are abstract. A new hardware
  implementation cannot silently do nothing.
- Startup validates the active module IDs, encoder channels, offsets, wheel
  diameter, gearing, gains, CAN bus, and current limits. Selecting CTRE while
  `kCtreConfigured` is false aborts before hardware is created.
- CTRE and REV configuration is applied with retries and every status is checked.
  A failed TalonFX/CANcoder/SparkMax configuration is reported and disables the
  affected module. Phoenix status signals are refreshed and checked every cycle,
  not just the encoder connection flag.
- REV motor CAN faults are sampled every 100 ms to keep the old robot's CAN load
  bounded; encoder angle health is still checked every cycle.
- A disconnected, non-finite, or implausibly jumping absolute encoder is never
  used for steering. The module neutralizes immediately, reports a fault after
  100 ms, and recovers after 200 ms of good readings.
- If one module is faulted or has a bad reading, only that module is neutralized;
  the other three continue in a conservative degraded mode capped at 1 m/s and
  1 rad/s. Two or more unavailable modules stop the drivetrain. The degraded
  mode is software-validated only and still needs physical testing.
- A disconnected or stale gyro forces robot-oriented driving, disables heading
  hold, and keeps the last valid heading for odometry. Field-oriented driving
  resumes after the gyro is healthy again.
- Faults and recoveries go to the Driver Station and to AdvantageKit:
  `Drive/FaultActive`, `Drive/FaultReason`, `Drive/FaultCount`,
  `Drive/RecoveryCount`, `Drive/HardwareMode`, `Drive/GyroFallbackActive`, and
  each module's `Healthy`, `ReadingValid`, `FaultReason`, and `RecoveryCount`.

## Autonomous

The dashboard **Auto** chooser defaults to **Do Nothing**. **Drive Forward**
resets the pose, drives robot-relative at 1 m/s for 2 seconds, and stops.

Add game autos in `commands/drivetrain/Autos.java` and register them in
`RobotContainer`.

## Vision

Vision is a foundation, not a camera implementation yet. A future camera calls:

```java
swerve.addVisionMeasurement(pose, timestampSeconds, stdDevs, tagCount, averageTagDistanceMeters);
```

Call it from the robot main loop only; the pose estimator is not thread-safe.
`Swerve` compares a delayed frame with WPILib's timestamped
`SwerveDrivePoseEstimator.sampleAt()` pose, not only with the current pose. It
filters stale, future, out-of-order, low-tag, invalid-covariance, outside-field,
and large-jump measurements, then logs `Vision/Accepted`,
`Vision/RejectedReason`, `Vision/HistoricalPose`, and accepted/rejected counts.
The 2025 field bounds live in `Constants.VisionConstants` and must be updated
for a new game. See `docs/VALIDATION.md`.

## Drive control

- Stick deadbands rescale continuously to the full usable range; there is no
  jump at the edge.
- Translation and rotation use a squared response for finer low-speed control.
  Full stick still reaches the configured maximum.
- The old `1 / 0.8` and `1 / 0.9` raw-input boost was removed.
- Translation and rotation acceleration limits remain separate and configurable.
  They use a fixed 20 ms step, so tests do not depend on wall-clock timing.
- A zero speed request holds the current wheel angles instead of snapping to the
  module-heading angles that WPILib's zero-speed kinematics returns.
- Non-finite stick or command inputs are treated as zero, not passed to motors.
- Requested chassis speeds are discretized over the 20 ms loop.
- Steering setpoints use the shortest path and cosine compensation; steering
  gains were not changed.
- Drive speed uses measured wheel velocity with feedforward and feedback, not a
  fixed percent output. The loop stays on the roboRIO; no SparkMax onboard
  velocity control is used.
- Heading hold captures the current heading when the turn stick is released and
  uses a separate P-only PID to hold it. Manual rotation always overrides it.
  Heading hold is disabled in robot-oriented mode, X-lock, disabled, and
  autonomous, and whenever field heading is unavailable.
- Start/Back resets the gyro and pose heading together and clears the held
  heading in Disabled, Teleop, and Test. If the gyro rejects the reset, the pose
  heading is left alone and the failure is reported.
- The REV path uses the WPILib steering PID; the CTRE path runs `PositionVoltage`
  steering in the TalonFX with a continuous target wrapped to the closest
  rotation.
- Teleop is blocked in Autonomous and Disabled.

## Repository layout

```text
frc/robot/
  Constants.java          robot-wide constants, hardware selection, startup validation
  DriverControls.java     the only physical controller/button mapping
  Robot.java              WPILib/AdvantageKit main loop
  RobotContainer.java     wiring, controls, auto chooser
  commands/drivetrain/    SwerveTeleOp, Autos, ResetHeading
  subsystems/drivetrain/  Swerve, SwerveModule, FaultMonitor, Gyro, IO implementations
```

The next robot can add `subsystems/superstructure/`, `subsystems/vision/`, and
their commands beside the drivetrain. Keep hardware behind an IO interface and
keep game-specific constants in `Constants`. The drivetrain has no dependency
on the superstructure, so it can be copied to a new SystemCore project and the
REV/CTRE IO implementation selected in one place.

For a new controller, update the WPILib/Gradle/vendor versions, the
`WPILOGWriter` path in `Robot.java` if the filesystem differs, and add the new
swerve IO implementation. The drivetrain control, tests, and package layout do
not depend on the roboRIO itself.

## Build, test, and deploy

```bash
./gradlew spotlessApply
./gradlew build          # compile, formatting, and all tests
./gradlew test           # simulation and logic tests
./gradlew simulateJava   # run on the host, not the devbox
./gradlew deploy         # only while the robot is disabled
```

The test suite covers forward driving, X-lock, module order/kinematics, gyro
sign and Start reset, angle wrapping, pose resets, steering optimization,
cosine compensation, stop behavior, disabled and initialization behavior,
teleop interruption, field- and robot-oriented driving at multiple headings,
heading-hold capture/hold/manual override/wraparound/reset, input shaping
(deadband continuity, squared response, full-stick maximum), periodic
drive-encoder stability, drive-voltage limits, vendor steering, measured
velocity control, the autonomous foundation, vision gating, delayed-vision
history, encoder disconnect/invalid/jump/recovery, rapid steering and sensor
noise rejection, gyro fallback/recovery, one- and two-module fault behavior,
semantic mode transitions, disabled heading reset, invalid/NaN inputs,
zero-speed wheel hold, startup validation, and CTRE steering wraparound.

GitHub Actions runs `spotlessCheck build` on every pull request and on `main`.
No robot or camera is needed for the offline test suite.

The robot logs measured versus requested chassis speeds, per-module
steering/speed error, loop time, battery voltage, all module inputs, health
flags, fault reasons, and recovery counts.

## AdvantageKit note

This repo uses **AdvantageKit 4.1.2** because 4.0.0 does not publish the
`akit-wpilibio` native for `linuxarm64`. The annotation processor in
`build.gradle` must match the vendor JSON version. See [docs/BUILD.md](docs/BUILD.md).

## Old-robot safety

CAN IDs, offsets, gearing, speed limits, and the REV steering PID are the last
known-working values. Do the blocks checks in
[docs/VALIDATION.md](docs/VALIDATION.md) before driving, including the analog
encoder voltage range and the gyro disconnect/fallback test. The CTRE path is not
hardware-tested until the new robot exists.