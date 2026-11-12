# Build and test environment

## Required tools

- **JDK 17**.
- **WPILib 2025.1.1** (GradleRIO), installed through the WPILib installer or VS Code extension.
- The included Gradle wrapper (`./gradlew`). Do not replace it with a system Gradle.
- Network access to `frcmaven.wpi.edu` on the first build, or a populated Gradle cache.
- Driver Station and a roboRIO for deployment.

## Commands

```bash
./gradlew build          # compile, Spotless, and all tests
./gradlew test           # simulation and logic tests only
./gradlew spotlessApply  # fix formatting
./gradlew simulateJava   # desktop simulation; run on the host, not the devbox
./gradlew deploy         # roboRIO; robot must be disabled
```

`./gradlew build` currently runs:

- `SwerveTest`: forward driving and X-lock in simulation.
- `SwerveControlTest`: module order/kinematics, gyro sign, pose resets, steering
  optimization, cosine compensation, stop behavior, vendor steering, teleop
  interruption, input shaping, heading hold, health/fault recovery, delayed
  vision, disabled reset, and measured velocity control.
- `PoseHistoryTest`: delayed-vision pose history, interpolation, eviction, and reset.
- `ConstantsTest`: startup wiring validation and the unfinished CTRE gate.
- `SwerveTeleOpTest`: input shaping.

GitHub Actions runs the same formatting, compile, and desktop test suite from
`.github/workflows/ci.yml` on every pull request and on `main`. The suite needs no
robot, camera, or Driver Station.

## AdvantageKit, ARM64, and the JDK

This build uses **JDK 17**, which is what WPILib/GradleRIO 2025 requires. Point
`JAVA_HOME` at the WPILib JDK 17. A newer JDK (21 or 25) can fail during Gradle
or annotation processing even when the source language level is 17; it is not a
supported substitute for this project. This is separate from the native
platform issue below.

The repo originally used **AdvantageKit 4.0.0**. Its `AdvantageKit.json` listed
`akit-wpilibio` platforms:

```text
linuxathena, linuxx86-64, osxuniversal, windowsx86-64
```

It did **not** list `linuxarm64`. That is a missing native for any ARM64 target
or ARM64 build host, and it causes dependency resolution to fail when the build
selects `linuxarm64`. It is not because the 2025 roboRIO target is ARM64: the
roboRIO still uses `linuxathena`. Do not describe the roboRIO 2 as an ARM64
target.

The fix is **AdvantageKit 4.1.2 or newer**, whose vendor JSON lists:

```text
linuxathena, linuxx86-64, linuxarm64, osxuniversal, windowsx86-64
```

This repo now uses:

- `vendordeps/AdvantageKit.json` version **4.1.2**.
- `build.gradle` annotation processor `org.littletonrobotics.akit:akit-autolog:4.1.2`.
- The `akit-java` / `akit-wpilibio` dependencies come from the vendor JSON.

The autolog processor and the vendor JSON must stay on the same version. If one
is updated, update the other in the same commit. If a future WPILib/AK update
reintroduces a platform mismatch, check `validPlatforms` in the vendor JSON
first, then look at the annotation processor version, then run a clean build.

A future SystemCore or other ARM64 controller also needs WPILib, Phoenix, REV,
and AdvantageKit natives for that platform. Having `linuxarm64` in AdvantageKit
is necessary but not sufficient; confirm every vendor before migrating.

### Verifying the fix

```bash
./gradlew clean build
./gradlew dependencies --configuration roborioRelease
```

`roborioRelease` should resolve the AdvantageKit native for `linuxathena`. For
an ARM64 host or target, the selected configuration must find `linuxarm64` in
`vendordeps/AdvantageKit.json`.

## Known environment notes

- **No USB stick:** AdvantageKit reports that it cannot open `/U/logs/`, but the
  robot continues running. NT4 logging still works. Plug in a USB or ignore the
  message.
- **No controller in simulation:** WPILib prints joystick-button warnings. They
  are expected when no controller is plugged into the simulator.
- **`simgui-ds.json`:** simulation window state, ignored by Git.
- **Build artifacts:** `build/` and `logs/` are ignored.

## Deploying

1. Connect the roboRIO and Driver Station.
2. Disable the robot.
3. Run `./gradlew deploy`.
4. Check the RioLog for `Robot program startup complete`.
5. If logging is configured, the `.wpilog` is under `/U/logs/`; without a USB,
   use AdvantageScope over NT4.

Never claim a deploy succeeded without checking the RioLog.