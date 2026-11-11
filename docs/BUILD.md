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
  interruption, vision gating, and measured velocity control.

## AdvantageKit and the linuxarm64 problem

The repo originally used **AdvantageKit 4.0.0**. Its `AdvantageKit.json`
listed `akit-wpilibio` platforms:

```text
linuxathena, linuxx86-64, osxuniversal, windowsx86-64
```

It did **not** list `linuxarm64`. A roboRIO 2 / ARM64 target therefore tried to
resolve an `akit-wpilibio` native that did not exist in 4.0.0, producing a
missing `linuxarm64` dependency error during dependency resolution or deploy.

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

### Verifying the fix

```bash
./gradlew clean build
./gradlew dependencies --configuration roborioRelease
```

`roborioRelease` should resolve the AdvantageKit native for the target platform.
For a linuxarm64 roboRIO, `vendordeps/AdvantageKit.json` must contain
`linuxarm64` in `validPlatforms`.

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