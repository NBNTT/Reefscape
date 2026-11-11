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

- REV drive and turn current limits are the old `40 A` values.
- If the turn motors are NEO 550s, check `turnTempCelsius` after a few minutes of
  steering; lower the turn limit if they get hot.
- The CTRE current limits are placeholders.

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
- Controls work in Teleop and Test, not in Autonomous or Disabled.
- `SwerveTest` checks forward motion and X-lock.
- `SwerveControlTest` checks the logic that is hard to test on hardware:
  module order, gyro sign, angle wrapping, pose resets, steering optimization,
  cosine compensation, stop/disabled/initialization behavior, teleop
  interruption, field- and robot-oriented driving at multiple headings,
  periodic drive-encoder stability, drive-voltage limits, vendor steering,
  measured velocity control, the autonomous foundation, and vision gating.
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
7. Release the sticks: the robot should stop and hold its wheel angles.
8. Drive a short distance and turn: `Drive/Pose` and `Drive/Heading` should
   move in the expected direction.
9. Check `Drive/ModuleX/SteeringErrorRad` and
   `Drive/ModuleX/DriveSpeedErrorMetersPerSec` while turning and accelerating.
10. Check motor temperatures and currents after a few minutes.

Only then take the robot off blocks.

## 5. Autonomous foundation

- The dashboard chooser defaults to **Do Nothing**.
- **Drive Forward** resets the pose, drives robot-relative at 1 m/s for 2 s,
  and stops.
- The auto command requires the drivetrain, so it interrupts the teleop default.
- `getAutonomousCommand()` returns the selected command; `Robot.autonomousInit()`
  schedules it and `teleopInit()` cancels it.

Add real game autos in `Autos` and the chooser in `RobotContainer`.

## 6. Vision fusion

Vision is not connected yet. When a camera is added, call:

```java
swerve.addVisionMeasurement(pose, timestampSeconds, stdDevs, tagCount, averageTagDistanceMeters);
```

- Timestamps must use the same clock as `Timer` / `RobotController`.
- Provide x, y, and heading standard deviations.
- The filter rejects invalid poses/timestamps, future timestamps, frames older
  than 0.5 s, out-of-order frames, too few tags, invalid covariance, and jumps
  over 1 m or 0.5 rad.
- `Vision/Accepted`, `Vision/RejectedReason`, `Vision/AcceptedCount`, and
  `Vision/RejectedCount` are logged for every measurement.
- Tune `Constants.VisionConstants` only after collecting real camera data.

## 7. Known limitations

- REV steering still uses the WPILib 50 Hz PID, not the SparkMax closed loop.
- CTRE steering/current values are placeholders until the Kraken robot is built.
- Vision has no camera implementation yet.
- The autonomous foundation only drives forward.
- Measured velocity gains are starting values, not a full SysId tune.