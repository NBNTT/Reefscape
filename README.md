# Team 8567 Drivetrain

This project is the drivetrain-only base for the 2024 test robot and the
2026-2027 robot.

## Current robot

The 2024 test robot uses the **REV MK4n** path, which is selected by
`Constants.kHardware = Hardware.REV_MK4N`. The CAN IDs, encoder channels,
offsets, gearing, and speed limits are the team's last known-working REV
values.

## Future robot

The future robot uses **CTRE MK5n** with Kraken X60 drive motors, Kraken X44
turn motors, and CANcoders. Before switching:

1. Fill in the CTRE IDs in `Constants.CAN`.
2. Fill in the CTRE module offsets, wheel diameter, and gear ratio in
   `Constants.DriveConstants.kCtreModuleConfigs` and `Constants.ModuleConstants`.
3. Change `Constants.kHardware` to `Hardware.CTRE_MK5N`.
4. Verify every wheel direction and offset on blocks before driving.

The CTRE code compiles today but is not tested on hardware.

## Driver controls

| Control | Action |
| --- | --- |
| Left stick | Drive (Teleop or Test mode) |
| Right stick X | Turn |
| Right bumper | Slow mode, with rumble |
| A | X-lock |
| Left bumper | Toggle field-oriented / robot-oriented |

The new Ultimate 3C+ controller's extra buttons are not mapped yet. To move the
robot-oriented toggle to one of them, change the `driver.leftBumper()` binding in
`RobotContainer`.

## Calibration

All module angles come from the absolute encoder. To check an offset:

1. Put the robot on blocks and point all four wheels straight forward.
2. Open AdvantageScope and look at `Drive/Module0` through `Drive/Module3`.
3. `turnAbsolutePositionRad` should be close to `0`. If it is not, adjust that
   module's `encoderOffsetRad` in `Constants.DriveConstants.kRevModuleConfigs`.
4. Check `absoluteEncoderConnected` is `true` for every module.
5. Command a slow forward motion on blocks; every
   `driveVelocityMetersPerSec` should be positive. If one is negative, that
   module's drive inversion or encoder direction is wrong.

## Build and test

```bash
./gradlew spotlessApply
./gradlew build
./gradlew test
./gradlew simulateJava       # run on the host, not the devbox
./gradlew deploy             # only while disabled
```

`./gradlew test` runs a simulation smoke test that drives the drivetrain
forward and checks the pose. It does not replace testing on the robot.