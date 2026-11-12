package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * All robot-wide constants for the drivetrain.
 *
 * <p>
 * There are two hardware sets:
 * <ul>
 * <li>REV MK4n: the old 2024 test robot. This is the active set.</li>
 * <li>CTRE MK5n: the future Kraken X60/X44 robot. Those values are placeholders
 * and are not active until {@link #kHardware} is changed.</li>
 * </ul>
 */
public final class Constants {
	private Constants() {
	}

	/**
	 * Selects the hardware implementation. The 2024 test robot uses the REV MK4n
	 * path; the 2026+ robot will use the CTRE MK5n path.
	 */
	public enum Hardware {
		REV_MK4N, CTRE_MK5N
	}

	/**
	 * The robot that is actually connected. Change this to
	 * {@link Hardware#CTRE_MK5N} only after the CTRE IDs, offsets, wheel size, and
	 * gear ratio below are filled in.
	 */
	public static final Hardware kHardware = Hardware.REV_MK4N;

	/**
	 * False until every CTRE value below is filled from the real MK5n robot.
	 * Selecting CTRE hardware while this is false aborts startup before any motor
	 * is created.
	 */
	public static final boolean kCtreConfigured = false;

	public static final class CAN {
		/** Empty means the roboRIO CAN bus; put a CANivore name here if used. */
		public static final String kCanBus = "";
		/** CAN ID of the Pigeon 2 gyro. */
		public static final int kPigeonId = 31;

		/*
		 * Old robot wiring. These are the last known-working REV values; do not change
		 * one without checking the actual robot.
		 */
		public static final int kRevFrontLeftDriveId = 10;
		public static final int kRevFrontLeftTurnId = 20;
		public static final int kRevFrontLeftEncoderChannel = 0;

		public static final int kRevFrontRightDriveId = 11;
		public static final int kRevFrontRightTurnId = 21;
		public static final int kRevFrontRightEncoderChannel = 1;

		public static final int kRevBackLeftDriveId = 12;
		public static final int kRevBackLeftTurnId = 22;
		public static final int kRevBackLeftEncoderChannel = 2;

		public static final int kRevBackRightDriveId = 13;
		public static final int kRevBackRightTurnId = 23;
		public static final int kRevBackRightEncoderChannel = 3;

		/*
		 * Future CTRE MK5n wiring. These are placeholders; fill them from the real
		 * robot before switching Constants.kHardware to CTRE_MK5N.
		 */
		public static final int kCtreFrontLeftDriveId = 1;
		public static final int kCtreFrontLeftTurnId = 5;
		public static final int kCtreFrontLeftEncoderId = 11;

		public static final int kCtreFrontRightDriveId = 2;
		public static final int kCtreFrontRightTurnId = 6;
		public static final int kCtreFrontRightEncoderId = 12;

		public static final int kCtreBackLeftDriveId = 3;
		public static final int kCtreBackLeftTurnId = 7;
		public static final int kCtreBackLeftEncoderId = 13;

		public static final int kCtreBackRightDriveId = 4;
		public static final int kCtreBackRightTurnId = 8;
		public static final int kCtreBackRightEncoderId = 14;
	}

	/**
	 * Everything one swerve module needs to know about its hardware.
	 *
	 * <p>
	 * The offset is applied in software, so the same calibration style works for
	 * the REV analog encoder and the CTRE CANcoder.
	 *
	 * @param name
	 *            FL, FR, BL, or BR; used in documentation and logs
	 * @param driveId
	 *            drive motor CAN ID
	 * @param turnId
	 *            steering motor CAN ID
	 * @param encoderId
	 *            REV analog channel or CTRE CANcoder CAN ID
	 * @param driveInverted
	 *            true if the drive motor must spin the other way
	 * @param turnInverted
	 *            true if the steering motor must spin the other way
	 * @param encoderInverted
	 *            true if the absolute encoder counts the other way
	 * @param encoderOffsetRad
	 *            angle added to the raw absolute encoder reading
	 * @param wheelDiameterMeters
	 *            wheel diameter in meters
	 * @param driveGearRatio
	 *            motor rotations per one wheel rotation
	 */
	public record SwerveModuleConfig(String name, int driveId, int turnId, int encoderId, boolean driveInverted,
			boolean turnInverted, boolean encoderInverted, double encoderOffsetRad, double wheelDiameterMeters,
			double driveGearRatio) {
		/** Meters the robot travels per one motor rotation. */
		public double driveRotToMeters() {
			return Math.PI * wheelDiameterMeters / driveGearRatio;
		}
	}

	public static final class ModuleConstants {
		/* REV MK4n: 3.75 inch wheel and 5.9:1 drive gearing. */
		public static final double kRevWheelDiameterMeters = Units.inchesToMeters(3.75);
		public static final double kRevDriveGearRatio = 5.9;

		/* CTRE MK5n placeholders; replace with the real module specs. */
		public static final double kCtreWheelDiameterMeters = Units.inchesToMeters(4.0);
		public static final double kCtreDriveGearRatio = 6.75;

		/* REV/old-robot steering PID. P is the old working value; tune on the robot. */
		public static final double kPTurning = 0.25;
		public static final double kITurning = 0.0;
		public static final double kDTurning = 0.0;

		/*
		 * CTRE vendor steering loop gains (volts per rotation). These are starting
		 * placeholders for the future robot; tune them on the Kraken robot.
		 */
		public static final double kCtreTurnKp = 10.0;
		public static final double kCtreTurnKi = 0.0;
		public static final double kCtreTurnKd = 0.0;
	}

	public static final class RevConstants {
		/*
		 * Old-robot REV hardware settings. These are active; change only with wiring.
		 */
		public static final int kDriveSmartCurrentLimitAmps = 40;
		public static final int kTurnSmartCurrentLimitAmps = 40;
		public static final double kVoltageCompensationVolts = 12.0;
		/* A motor controller may be slow to answer right after power-up. */
		public static final int kConfigRetries = 3;
		public static final double kConfigRetryDelaySeconds = 0.02;
		/* REV CAN fault flags are sampled every 100 ms at 50 Hz. */
		public static final int kFaultPollFrames = 5;
		/* The analog encoder is assumed to span 0 V to the measured 5 V rail. */
		public static final double kEncoderMinVolts = 0.01;
		public static final double kEncoderMaxMarginVolts = 0.01;
		public static final double kMinReferenceVoltageVolts = 4.5;
	}

	public static final class CtreConstants {
		/* Future MK5n hardware settings; placeholders until the real robot exists. */
		public static final double kDriveSupplyCurrentLimitAmps = 40.0;
		public static final double kDriveStatorCurrentLimitAmps = 80.0;
		public static final double kTurnSupplyCurrentLimitAmps = 40.0;
		public static final double kTurnStatorCurrentLimitAmps = 80.0;
		public static final double kPeakForwardVoltage = 12.0;
		public static final double kPeakReverseVoltage = -12.0;
		/* 50 Hz is enough for the 50 Hz robot loop and keeps CAN load down. */
		public static final double kStatusUpdateHz = 50.0;
		public static final int kConfigRetries = 3;
		public static final double kConfigRetryDelaySeconds = 0.02;
		public static final double kConfigTimeoutSeconds = 0.1;
	}

	public static final class HealthConstants {
		/* Invalid frames before a module or the gyro is reported failed. */
		public static final int kEncoderFaultDebounceFrames = 5;
		public static final int kGyroFaultDebounceFrames = 5;
		/* Consecutive good frames required before driving resumes after a fault. */
		public static final int kEncoderRecoveryFrames = 10;
		public static final int kGyroRecoveryFrames = 10;
		/*
		 * A steering angle jump this large in one 20 ms cycle is not physically
		 * plausible. A new angle must stay stable for the confirm count before it is
		 * accepted, so a single noisy sample cannot steer the module.
		 */
		public static final double kMaxAngleJumpRad = 1.5;
		public static final int kAngleJumpConfirmFrames = 10;
	}

	public static final class DriveConstants {
		/* Old robot measurement. Change these if the chassis is different. */
		public static final double kTrackWidth = Units.inchesToMeters(20.75);
		public static final double kWheelBase = Units.inchesToMeters(20.75);

		/*
		 * Converts wheel motion into robot motion. Module order is FL, FR, BL, BR and
		 * must match kRevModuleConfigs/kCtreModuleConfigs and Swerve.
		 */
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

		/* Physical top speed. Drive output is a fraction of this value. */
		public static final double kMaxSpeedMetersPerSecond = 4.5;
		/* Old teleop turn cap. The physical maximum is 3*pi rad/s. */
		public static final double kMaxAngularSpeedRadiansPerSecond = 4.5;
		/* The robot loop runs at 50 Hz; used to correct curved motion. */
		public static final double kLoopPeriodSeconds = 0.02;
		/* Slew-rate limits, used so the robot starts moving smoothly. */
		public static final double kMaxAccelerationMetersPerSecondSquared = 13.5;
		public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 13.5;

		/*
		 * Drive velocity control: measured wheel speed is fed back instead of trusting
		 * a fixed percent output. kS and kV are calibrated values; kV is seeded from
		 * the 12 V / top-speed estimate. kI and kD are zero so the loop can safely run
		 * twice per cycle.
		 */
		public static final double kDriveKp = 0.5;
		public static final double kDriveKi = 0.0;
		public static final double kDriveKd = 0.0;
		public static final double kDriveKsVolts = 0.0;
		public static final double kDriveKvVoltsPerMps = 12.0 / kMaxSpeedMetersPerSecond;

		/*
		 * Order is FL, FR, BL, BR and must match kDriveKinematics. The offsets are the
		 * old calibration record: they make each wheel read 0 when it points straight
		 * forward. Do not "simplify" the sums unless you recalibrate.
		 */
		public static final List<SwerveModuleConfig> kRevModuleConfigs = List.of(
				new SwerveModuleConfig("FL", CAN.kRevFrontLeftDriveId, CAN.kRevFrontLeftTurnId,
						CAN.kRevFrontLeftEncoderChannel, true, true, false, -2.854 - 0.494 - 0.548 - 2.987 - 0.02,
						ModuleConstants.kRevWheelDiameterMeters, ModuleConstants.kRevDriveGearRatio),
				new SwerveModuleConfig("FR", CAN.kRevFrontRightDriveId, CAN.kRevFrontRightTurnId,
						CAN.kRevFrontRightEncoderChannel, true, true, false, -2.231 + 0.060 - 0.05 - 2.83 + Math.PI,
						ModuleConstants.kRevWheelDiameterMeters, ModuleConstants.kRevDriveGearRatio),
				new SwerveModuleConfig("BL", CAN.kRevBackLeftDriveId, CAN.kRevBackLeftTurnId,
						CAN.kRevBackLeftEncoderChannel, true, true, false, 3.022 + 0.115 - 0.092 - 0.01,
						ModuleConstants.kRevWheelDiameterMeters, ModuleConstants.kRevDriveGearRatio),
				new SwerveModuleConfig("BR", CAN.kRevBackRightDriveId, CAN.kRevBackRightTurnId,
						CAN.kRevBackRightEncoderChannel, true, true, false,
						0.349 + 0.324 + 1.029 + 0.672 - 1.961 - 0.082 + 0.125 + 0.07,
						ModuleConstants.kRevWheelDiameterMeters, ModuleConstants.kRevDriveGearRatio));

		/* Same FL, FR, BL, BR order. These are placeholders for the future robot. */
		public static final List<SwerveModuleConfig> kCtreModuleConfigs = List.of(
				new SwerveModuleConfig("FL", CAN.kCtreFrontLeftDriveId, CAN.kCtreFrontLeftTurnId,
						CAN.kCtreFrontLeftEncoderId, true, true, false, 0.0, ModuleConstants.kCtreWheelDiameterMeters,
						ModuleConstants.kCtreDriveGearRatio),
				new SwerveModuleConfig("FR", CAN.kCtreFrontRightDriveId, CAN.kCtreFrontRightTurnId,
						CAN.kCtreFrontRightEncoderId, true, true, false, 0.0, ModuleConstants.kCtreWheelDiameterMeters,
						ModuleConstants.kCtreDriveGearRatio),
				new SwerveModuleConfig("BL", CAN.kCtreBackLeftDriveId, CAN.kCtreBackLeftTurnId,
						CAN.kCtreBackLeftEncoderId, true, true, false, 0.0, ModuleConstants.kCtreWheelDiameterMeters,
						ModuleConstants.kCtreDriveGearRatio),
				new SwerveModuleConfig("BR", CAN.kCtreBackRightDriveId, CAN.kCtreBackRightTurnId,
						CAN.kCtreBackRightEncoderId, true, true, false, 0.0, ModuleConstants.kCtreWheelDiameterMeters,
						ModuleConstants.kCtreDriveGearRatio));

		/*
		 * A true X: front-left and back-right wheels on one diagonal, front-right and
		 * back-left on the other. Speed is zero; the steering PID moves the wheels into
		 * position.
		 */
		public static SwerveModuleState[] crossModuleStates() {
			return new SwerveModuleState[]{new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))};
		}
	}

	public static final class HeadingConstants {
		/*
		 * Heading-hold PID. P-only and conservative; output is rad/s. Keep these
		 * separate from the steering PID.
		 */
		public static final double kKp = 2.0;
		public static final double kKi = 0.0;
		public static final double kKd = 0.0;
		/* Normalized turn input below this is treated as "released". */
		public static final double kManualTurnThreshold = 0.05;
	}

	public static final class VisionConstants {
		/* Reject anything older than this; a stale pose can drag the robot. */
		public static final double kMaxAgeSeconds = 0.5;
		/* One tag is enough for a drivetrain-only foundation; raise for competition. */
		public static final int kMinTags = 1;
		/* Reject a measurement that jumps farther than this from the current pose. */
		public static final double kMaxTranslationInnovationMeters = 1.0;
		public static final double kMaxRotationInnovationRadians = 0.5;
		/* 2025 Reefscape field bounds; update these for each new game. */
		public static final double kFieldLengthMeters = 16.54175;
		public static final double kFieldWidthMeters = 8.21055;
		/* About two seconds of pose history at 50 Hz for delayed vision checks. */
		public static final int kPoseHistoryCapacity = 100;
	}

	public static final class OIConstants {
		/* Driver controller port on the Driver Station. */
		public static final int kDriverControllerPort = 0;
		/* Stick inputs smaller than this are treated as zero. */
		public static final double kDeadband = 0.1;
		/* Right bumper scales the drivetrain to this fraction of full speed. */
		public static final double kSlowModeFactor = 0.33;
		/* Rumble strength while slow mode is held. */
		public static final double kRumble = 0.025;
	}

	/** The wiring set for the selected {@link #kHardware}. */
	public static List<SwerveModuleConfig> activeModuleConfigs() {
		return kHardware == Hardware.CTRE_MK5N ? DriveConstants.kCtreModuleConfigs : DriveConstants.kRevModuleConfigs;
	}

	/**
	 * Fails startup with a readable message if the active hardware set is not
	 * complete. Called by {@link frc.robot.subsystems.drivetrain.Swerve} before any
	 * motor or encoder object is created.
	 */
	public static void validateActiveHardware() {
		validateHardware(kHardware, activeModuleConfigs(), kCtreConfigured);
	}

	/**
	 * Testable validation core. Package-private so tests can pass broken configs
	 * without changing the real robot constants.
	 */
	static void validateHardware(Hardware hardware, List<SwerveModuleConfig> configs, boolean ctreConfigured) {
		require(hardware != null, "hardware selection is null");
		require(configs != null && configs.size() == 4,
				"expected four swerve module configs, got " + (configs == null ? "null" : configs.size()));
		require(DriveConstants.kTrackWidth > 0.0, "track width must be positive");
		require(DriveConstants.kWheelBase > 0.0, "wheelbase must be positive");
		require(DriveConstants.kMaxSpeedMetersPerSecond > 0.0, "max speed must be positive");
		require(DriveConstants.kMaxAngularSpeedRadiansPerSecond > 0.0, "max angular speed must be positive");
		require(DriveConstants.kLoopPeriodSeconds > 0.0, "loop period must be positive");
		require(DriveConstants.kMaxAccelerationMetersPerSecondSquared > 0.0, "max acceleration must be positive");
		require(DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared > 0.0,
				"max angular acceleration must be positive");
		require(isFiniteNonNegative(DriveConstants.kDriveKp), "drive kP must be finite and non-negative");
		require(isFiniteNonNegative(DriveConstants.kDriveKi), "drive kI must be finite and non-negative");
		require(isFiniteNonNegative(DriveConstants.kDriveKd), "drive kD must be finite and non-negative");
		require(isFiniteNonNegative(DriveConstants.kDriveKsVolts), "drive kS must be finite and non-negative");
		require(isFiniteNonNegative(DriveConstants.kDriveKvVoltsPerMps), "drive kV must be finite and non-negative");
		require(isFiniteNonNegative(ModuleConstants.kPTurning), "steering kP must be finite and non-negative");
		require(isFiniteNonNegative(ModuleConstants.kITurning), "steering kI must be finite and non-negative");
		require(isFiniteNonNegative(ModuleConstants.kDTurning), "steering kD must be finite and non-negative");
		require(isFiniteNonNegative(ModuleConstants.kCtreTurnKp), "CTRE steering kP must be finite and non-negative");
		require(isFiniteNonNegative(ModuleConstants.kCtreTurnKi), "CTRE steering kI must be finite and non-negative");
		require(isFiniteNonNegative(ModuleConstants.kCtreTurnKd), "CTRE steering kD must be finite and non-negative");
		require(isFiniteNonNegative(HeadingConstants.kKp), "heading kP must be finite and non-negative");
		require(isFiniteNonNegative(HeadingConstants.kKi), "heading kI must be finite and non-negative");
		require(isFiniteNonNegative(HeadingConstants.kKd), "heading kD must be finite and non-negative");
		require(RevConstants.kDriveSmartCurrentLimitAmps > 0, "REV drive current limit must be positive");
		require(RevConstants.kTurnSmartCurrentLimitAmps > 0, "REV turn current limit must be positive");
		require(RevConstants.kVoltageCompensationVolts > 0.0, "REV voltage compensation must be positive");
		require(RevConstants.kConfigRetries > 0, "REV config retries must be positive");
		require(RevConstants.kConfigRetryDelaySeconds >= 0.0, "REV retry delay must not be negative");
		require(RevConstants.kFaultPollFrames > 0, "REV fault poll interval must be positive");
		require(RevConstants.kEncoderMinVolts >= 0.0 && RevConstants.kEncoderMaxMarginVolts >= 0.0,
				"REV encoder voltage margins must not be negative");
		require(RevConstants.kMinReferenceVoltageVolts > 0.0 && RevConstants.kMinReferenceVoltageVolts < 5.5,
				"REV encoder reference voltage threshold is implausible");
		require(CtreConstants.kDriveSupplyCurrentLimitAmps > 0.0, "CTRE drive supply limit must be positive");
		require(CtreConstants.kDriveStatorCurrentLimitAmps > 0.0, "CTRE drive stator limit must be positive");
		require(CtreConstants.kTurnSupplyCurrentLimitAmps > 0.0, "CTRE turn supply limit must be positive");
		require(CtreConstants.kTurnStatorCurrentLimitAmps > 0.0, "CTRE turn stator limit must be positive");
		require(CtreConstants.kPeakForwardVoltage > 0.0, "CTRE peak forward voltage must be positive");
		require(CtreConstants.kPeakReverseVoltage < 0.0, "CTRE peak reverse voltage must be negative");
		require(CtreConstants.kStatusUpdateHz > 0.0, "CTRE status update rate must be positive");
		require(CtreConstants.kConfigRetries > 0, "CTRE config retries must be positive");
		require(CtreConstants.kConfigRetryDelaySeconds >= 0.0, "CTRE retry delay must not be negative");
		require(CtreConstants.kConfigTimeoutSeconds > 0.0, "CTRE config timeout must be positive");
		require(CAN.kCanBus != null, "CAN bus name is null");
		require(CAN.kPigeonId > 0, "Pigeon CAN ID must be positive");

		if (hardware == Hardware.CTRE_MK5N) {
			require(ctreConfigured, "CTRE hardware is selected but Constants.kCtreConfigured is false; "
					+ "fill the CTRE IDs, offsets, wheel size, gearing, current limits, and gains first");
		}

		Set<Integer> canIds = new HashSet<>();
		canIds.add(CAN.kPigeonId);
		Set<Integer> analogChannels = new HashSet<>();
		Set<String> names = new HashSet<>();
		for (SwerveModuleConfig config : configs) {
			require(config != null, "module config is null");
			require(config.name() != null && !config.name().isBlank(), "module name is blank");
			require(names.add(config.name()), "duplicate module name " + config.name());
			require(config.driveId() > 0, config.name() + " drive CAN ID must be positive");
			require(config.turnId() > 0, config.name() + " turn CAN ID must be positive");
			require(canIds.add(config.driveId()), "duplicate CAN ID " + config.driveId() + " at " + config.name());
			require(canIds.add(config.turnId()), "duplicate CAN ID " + config.turnId() + " at " + config.name());
			if (hardware == Hardware.CTRE_MK5N) {
				require(config.encoderId() > 0, config.name() + " CANcoder ID must be positive");
				require(canIds.add(config.encoderId()),
						"duplicate CAN ID " + config.encoderId() + " at " + config.name());
			} else {
				require(config.encoderId() >= 0 && config.encoderId() <= 3,
						config.name() + " REV analog encoder channel must be 0-3");
				require(analogChannels.add(config.encoderId()),
						"duplicate analog encoder channel " + config.encoderId());
			}
			require(Double.isFinite(config.encoderOffsetRad()), config.name() + " encoder offset must be finite");
			require(config.wheelDiameterMeters() > 0.05 && config.wheelDiameterMeters() < 0.3,
					config.name() + " wheel diameter is implausible");
			require(config.driveGearRatio() > 0.1 && config.driveGearRatio() < 100.0,
					config.name() + " drive gear ratio is implausible");
		}
	}

	private static boolean isFiniteNonNegative(double value) {
		return Double.isFinite(value) && value >= 0.0;
	}

	private static void require(boolean condition, String message) {
		if (!condition) {
			throw new IllegalStateException("Hardware configuration error: " + message);
		}
	}
}
