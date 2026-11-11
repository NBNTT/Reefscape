# Calibration, simulation, deployment, and physical validation

This is the drivetrain-only checklist. Do the steps in order; do not skip the
blocks checks.

## 1. Calibration

### Wheel radius and gearing

`Constants.ModuleConstants` and each `SwerveModuleConfig` describe the real
module. The current REV values are the last known-working MK4n values:

- 3.75 inch wheel.
- 5.9:1 drive gearing.
- Calibrated absolute-encoder offsets in `kRevModuleConfigs`.

If a wheel or module changes, update the wheel diameter, gearing, and offset.
Do not "simplify" the offset sums without recalibrating.

### Steering offset

1. Put the robot on blocks.
2. Point all four wheels straight forward.
3. Open AdvantageScope and read `Drive/Module0..3/turnAbsolutePositionRad`.
4. Each should be near `0`. Adjust that module's `encoderOffsetRad` until it is.
5. Confirm `Drive/Module0..3/absoluteEncoderConnected` is `true`.

### Analog encoder voltage range

The REV analog encoder is assumed to output **0 V to the measured 5 V rail**. The
code logs `absoluteEncoderVoltage` and `absoluteEncoderReferenceVoltage` and
rejects readings at the rails or with a 5 V rail below 4.5 V.

1. With the robot on blocks, rotate one wheel by hand through a full turn.
2. Confirm the logged voltage moves smoothly over most of the 0–5 V range and
   does not sit at 0 V or at the reference voltage.
3. Repeat for all four modules. If a sensor is 3.3 V, the angle scale is wrong;
   change the sensor or the conversion before driving.

### Drive direction

1. Command a slow forward motion on blocks.
2. Every `Drive/Module0..3/driveVelocityMetersPerSec` should be positive.
3. If one is negative, flip that module's `driveInverted`.
4. If one steers the wrong way, flip `turnInverted`.

### Drive velocity gains

The robot now uses measured wheel velocity, not a fixed percent output.

- `kDriveKvVoltsPerMps` is seeded as `12 / top speed`. A proper value comes
  from a push test or SysId.
- `kDriveKsVolts` is currently `0`. A small value compensates friction but can
  make a stopped robot creep; tune it only with the robot on blocks.
- `kDriveKp` is a starting feedback gain. Increase until the wheel tracks
  `Drive/Module0..3/DriveSpeedErrorMetersPerSec`, then back off if it oscillates.

### Steering gains

- `kPTurning` is the old working `0.25` for the REV path.
- The CTRE path uses `PositionVoltage` with `kCtreTurnKp/Ki/Kd` placeholders.
  Tune those on the new robot only.
- Watch `Drive/Module0..3/SteeringErrorRad` during a turn. It should shrink
  quickly without oscillating.

### Current limits and temperatures

- The old robot uses REV NEO drive and turn motors; the `40 A` limits are
  intentional and live in `Constants.RevConstants`. Check `turnTempCelsius`
  after a few minutes of steering.
- The analog absolute encoder is assumed to use a 0–5 V full scale. Confirm the
  sensor with the voltage-range check above.
- The CTRE current limits, voltage limits, status rate, and retry values live in
  `Constants.CtreConstants` and remain placeholders.

### Before enabling the CTRE path

Do not set `Constants.kCtreConfigured = true` until every item is filled from the
real MK5n robot:

- CAN IDs for four drive motors, four turn motors, four CANcoders, and the
  Pigeon 2, plus the CANivore name if one is used.
- CANcoder IDs, magnet offsets, and sensor direction.
- Wheel diameter and drive gear ratio.
- Drive and turn motor inversion and sensor direction.
- Drive and turn current limits and voltage limits in `Constants.CtreConstants`.
- CTRE steering gains in `Constants.ModuleConstants`.

`Constants.validateActiveHardware()` and the `SwerveModuleIOTalonFX` constructor
both refuse to run with CTRE selected while `kCtreConfigured` is false. The CTRE
code is still unvalidated until it runs on the new robot.

### Track width and wheelbase

`Constants.DriveConstants.kTrackWidth` and `kWheelBase` are the old 20.75 inch
measurements. If the chassis is different, change both before trusting odometry
or field-oriented driving.

## 2. Simulation

```bash
./gradlew simulateJava
```

- Simulation uses `SwerveModuleIOSim` and `GyroIOSim`; no REV or CTRE hardware
  is opened.
- Driving, X-lock, and the mode toggle work in Teleop and Test, not in Autonomous
  or Disabled. Start/Back heading reset also works while Disabled.
- `SwerveTest` checks forward motion and X-lock.
- `SwerveControlTest` checks the logic that is hard to test on hardware:
  module order, gyro sign, Start reset, angle wrapping, pose resets, steering
  optimization, cosine compensation, stop/disabled/initialization behavior,
  teleop interruption, field- and robot-oriented driving at multiple headings,
  heading-hold capture/hold/manual override/wraparound/reset, periodic
  drive-encoder stability, drive-voltage limits, vendor steering, measured
  velocity control, the autonomous foundation, vision gating and delayed-vision
  history, encoder disconnect/invalid/jump/recovery, gyro fallback/recovery,
  module-fault safe stop, semantic mode transitions, disabled heading reset,
  invalid/NaN inputs, zero-speed wheel hold, and CTRE steering wraparound.
- `ConstantsTest` checks startup validation, including the unfinished CTRE gate.
- `SwerveTeleOpTest` checks deadband continuity, squared response, sign
  preservation, and full-stick reach.
- Simulation does not prove motor directions, encoder offsets, current limits,
  or carpet behavior.

## 3. Deployment

1. Connect the roboRIO and Driver Station.
2. Disable the robot.
3. Run `./gradlew deploy`.
4. Confirm `Robot program startup complete` in the RioLog.
5. Open AdvantageScope and check:
   - `Drive/LoopTimeSeconds` near `0.02`.
   - `Drive/BatteryVoltage`.
   - `Drive/RequestedChassisSpeeds`, `Drive/DiscretizedChassisSpeeds`, and
     `Drive/MeasuredChassisSpeeds`.
   - `Drive/Module0..3/*` inputs and setpoints.
   - `Drive/HardwareMode`, `Drive/FaultActive`, `Drive/FaultReason`,
     `Drive/GyroHealthy`, and `Drive/GyroFallbackActive`.
   - `Vision/*` only after a camera is connected.

If there is no USB stick, `[AdvantageKit] Failed to open output log file` is
expected and not fatal. NT4 still streams data.

## 4. Physical robot validation

Do this on blocks first.

1. Enable **Teleop** or **Test**.
2. Wheels straight forward: all absolute angles near 0 and connected.
3. Slow forward: all drive velocities positive.
4. A: X-lock; wheels should move to a true X and hold.
5. Right bumper: slow mode and right-side rumble.
6. Left bumper: field-oriented / robot-oriented toggle.
7. Start (and Back): the current direction becomes field-forward; the pose
   heading and gyro both read zero. Back was the original 2025 binding.
8. Release the sticks: the robot should stop and hold the wheel angles it already
   has; the wheels must not snap toward 45 degrees.
9. Drive a short distance and turn: `Drive/Pose` and `Drive/Heading` should
   move in the expected direction.
10. Release the turn stick: heading hold should keep the current direction while
    translating. Push the robot slightly and confirm it corrects.
11. Command manual rotation: it must override heading hold immediately.
12. Select robot-oriented mode: heading hold must be inactive.
13. Check `Drive/ModuleX/SteeringErrorRad` and
    `Drive/ModuleX/DriveSpeedErrorMetersPerSec` while turning and accelerating.
14. Check motor temperatures and currents after a few minutes.
15. Unplug one absolute encoder while the robot is disabled. Confirm the Driver
    Station error, `Drive/FaultReason`, and that the module reports
    `ReadingValid = false`; enabling must not drive. Reconnect it and confirm the
    module recovers after about 200 ms.
16. Unplug the Pigeon 2 (or power it down) while disabled. Confirm
    `Drive/GyroFallbackActive` is true and that Teleop drives robot-oriented with
    heading hold off. Reconnect it and confirm field-oriented driving returns.
17. Check `Drive/LoopTimeSeconds` stays near `0.02` and watch CPU usage in the
    Driver Station. If loop time is high, use AdvantageScope/AdvantageKit replay
    to find the slow code before optimizing.
18. Use Phoenix Tuner X to check CAN utilization and status-signal rates. Reduce
    rates only if measured bus load requires it.
19. Confirm `Drive/HardwareMode` says `REV_MK4N`.

Only then take the robot off blocks.

## 5. Heading hold and input shaping

- Deadband inputs are zero; just outside the deadband the command ramps
  continuously from zero. There is no jump.
- Translation and rotation use a squared response. Half stick is less than half
  speed; full stick reaches the configured maximum.
- Heading hold captures the heading when the turn stick is released. It is
  P-only (`Constants.HeadingConstants`) and separate from the steering PID.
- Manual rotation always overrides heading hold.
- Start/Back resets the gyro and pose heading together and clears the held
  target; `Drive/HeadingResetCount` is logged. Both buttons work in Disabled,
  Teleop, and Test, and are intentionally not bound while Autonomous is running.
  If the gyro rejects the reset, the pose heading is left alone and the failure
  is reported.
- Heading hold is disabled in robot-oriented mode, X-lock, disabled, and
  autonomous, and whenever the gyro is unhealthy or its reading is invalid.
- Steering PID gains, deadbands, and algorithms were intentionally not changed.
  Tune steering only on the robot if evidence shows a need.

## 6. Autonomous foundation

- The dashboard chooser defaults to **Do Nothing**.
- **Drive Forward** resets the pose, drives robot-relative at 1 m/s for 2 s,
  and stops.
- The auto command requires the drivetrain, so it interrupts the teleop default.
- `getAutonomousCommand()` returns the selected command; `Robot.autonomousInit()`
  schedules it and `teleopInit()` cancels it.

Add real game autos in `Autos` and the chooser in `RobotContainer`.

## 7. Vision fusion

Vision is not connected yet. When a camera is added, call:

```java
swerve.addVisionMeasurement(pose, timestampSeconds, stdDevs, tagCount, averageTagDistanceMeters);
```

- Call only from the robot main loop. The pose estimator is not thread-safe; a
  camera callback thread must queue the measurement.
- Timestamps must use the same clock as `Timer` / `RobotController`.
- Provide x, y, and heading standard deviations.
- Delayed frames are compared with the pose history at the frame timestamp, not
  just the current pose. `Vision/HistoricalPose` and
  `Vision/CurrentTranslationInnovationMeters` show both checks.
- The filter rejects invalid poses/timestamps, future timestamps, frames older
  than 0.5 s, out-of-order frames, too few tags, invalid covariance, poses
  outside the field, missing pose history, and jumps over 1 m or 0.5 rad.
- The field bounds are the 2025 Reefscape values in
  `Constants.VisionConstants`; update them for the new game.
- `Vision/Accepted`, `Vision/RejectedReason`, `Vision/AcceptedCount`, and
  `Vision/RejectedCount` are logged for every measurement.
- Tune `Constants.VisionConstants` only after collecting real camera data.

## 8. Known limitations

- REV steering still uses the WPILib 50 Hz PID, not the SparkMax closed loop.
- CTRE steering/current values are placeholders until the Kraken robot is built.
  The CTRE implementation has not been run on hardware.
- Three-wheel degraded driving is intentionally not implemented. A module fault
  stops the whole drivetrain.
- Vision has no camera implementation yet.
- The autonomous foundation only drives forward.
- Measured velocity gains are starting values, not a full SysId tune.