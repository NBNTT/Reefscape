package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Hardware;
import frc.robot.Constants.HealthConstants;
import frc.robot.Constants.SwerveModuleConfig;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The drivetrain. This is the class the rest of the robot talks to.
 *
 * <p>
 * It owns the four swerve modules, the gyro, and the pose estimate. Module
 * order is FL, FR, BL, BR and must match
 * {@link DriveConstants#kDriveKinematics}.
 *
 * <p>
 * Hardware differences are hidden behind {@link SwerveModuleIO}: the same code
 * runs the old REV robot, the future CTRE robot, and simulation.
 */
public class Swerve extends SubsystemBase {
	private final SwerveModule[] modules = new SwerveModule[4];
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final SwerveDrivePoseEstimator poseEstimator;
	private final PoseHistory poseHistory = new PoseHistory(VisionConstants.kPoseHistoryCapacity);
	/** Used only to report the real loop time in AdvantageScope. */
	private double lastTimestampSeconds = Timer.getFPGATimestamp();
	/** Vision measurements older than the last accepted one are ignored. */
	private double lastAcceptedVisionTimestampSeconds = -1.0;
	private int acceptedVisionCount = 0;
	private int rejectedVisionCount = 0;
	private int headingResetCount = 0;

	/*
	 * Gyro health. A bad frame immediately disables field-oriented driving; the
	 * debounced flag is what the Driver Station sees.
	 */
	private boolean gyroHealthy = false;
	private boolean gyroReadingValid = false;
	private int gyroInvalidFrames = 0;
	private int gyroRecoveryFrames = 0;
	private int gyroRecoveryCount = 0;
	private double lastValidYawRad = 0.0;
	private boolean hasValidYaw = false;

	/* Persistent fault reporting. */
	private boolean faultActive = false;
	private String faultSummary = "";
	private int faultCount = 0;
	private int recoveryCount = 0;

	public Swerve() {
		// Reject incomplete wiring before opening any motor or encoder.
		Constants.validateActiveHardware();
		List<SwerveModuleConfig> configs = Constants.activeModuleConfigs();

		// Build one module per wheel. Simulation uses the same SwerveModule logic
		// but swaps the hardware layer for SwerveModuleIOSim.
		for (int i = 0; i < modules.length; i++) {
			SwerveModuleIO io;
			if (RobotBase.isSimulation()) {
				io = new SwerveModuleIOSim();
			} else if (Constants.kHardware == Hardware.CTRE_MK5N) {
				io = new SwerveModuleIOTalonFX(configs.get(i));
			} else {
				io = new SwerveModuleIOSparkMax(configs.get(i));
			}
			modules[i] = new SwerveModule(i, io);
		}

		// The real robot uses the Pigeon 2. Simulation integrates the measured
		// chassis rotation so field-oriented driving still behaves normally.
		gyroIO = RobotBase.isSimulation()
				? new GyroIOSim(this::getMeasuredAngularVelocity)
				: new GyroIOPigeon2(CAN.kPigeonId, CAN.kCanBus);
		poseEstimator = createPoseEstimator();
		poseHistory.reset(Timer.getFPGATimestamp(), getPose());
	}

	/**
	 * Test/simulation constructor. Lets {@link SwerveTest} inject simulated
	 * hardware without touching a real motor or gyro.
	 */
	Swerve(SwerveModuleIO[] moduleIOs, GyroIO gyroIO) {
		this.gyroIO = gyroIO;
		for (int i = 0; i < modules.length; i++) {
			modules[i] = new SwerveModule(i, moduleIOs[i]);
		}
		poseEstimator = createPoseEstimator();
		poseHistory.reset(Timer.getFPGATimestamp(), getPose());
	}

	/**
	 * Holds each module at its first valid angle and reads the gyro once so the
	 * starting pose uses real hardware instead of zeros.
	 */
	private SwerveDrivePoseEstimator createPoseEstimator() {
		for (SwerveModule module : modules) {
			module.holdCurrentAngle();
		}
		gyroIO.updateInputs(gyroInputs);
		boolean valid = gyroInputs.connected && Double.isFinite(gyroInputs.yawRad);
		gyroReadingValid = valid;
		gyroHealthy = valid;
		hasValidYaw = valid;
		lastValidYawRad = valid ? gyroInputs.yawRad : 0.0;
		return new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, new Rotation2d(effectiveYawRad()),
				getModulePositions(), new Pose2d());
	}

	@Override
	public void periodic() {
		// 1) Read every module and validate its sensors.
		for (SwerveModule module : modules) {
			module.periodic();
		}

		// 2) Read the gyro and decide whether field heading is usable.
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
		updateGyroHealth();

		// 3) Update the pose from wheel motion and the best available heading.
		poseEstimator.update(new Rotation2d(effectiveYawRad()), getModulePositions());

		// 4) Never leave motor outputs latched while the robot is disabled.
		if (DriverStation.isDisabled()) {
			stopModules();
		}

		// 5) Report health to the Driver Station and AdvantageKit.
		updateFaultReporting();

		// 6) Log what the robot actually did this cycle.
		double now = Timer.getFPGATimestamp();
		Logger.recordOutput("Drive/LoopTimeSeconds", now - lastTimestampSeconds);
		lastTimestampSeconds = now;
		Logger.recordOutput("Drive/BatteryVoltage", RobotController.getBatteryVoltage());
		Logger.recordOutput("Drive/MeasuredChassisSpeeds", getRobotRelativeSpeeds());
		Logger.recordOutput("Drive/Pose", getPose());
		Logger.recordOutput("Drive/Heading", getHeading());
		Logger.recordOutput("Drive/MeasuredStates", getModuleStates());
		Logger.recordOutput("Drive/DriveDisabled", !isDrivetrainHealthy());
		Logger.recordOutput("Drive/DriveBlockedReason", blockedReason());
		poseHistory.add(now, getPose());
	}

	/** FL, FR, BL, BR. The order must stay the same as the kinematics. */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getPosition();
		}
		return positions;
	}

	/** FL, FR, BL, BR. Used for logging and by PathPlanner later. */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/** What the robot is actually doing, relative to its own front. */
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
	}

	/** Turns a robot-relative speed request into four wheel commands. */
	public void setModuleStates(ChassisSpeeds chassisSpeeds) {
		// While the robot is rotating, the wheels must follow a slight curve over
		// the next 20 ms. Discretize corrects for that instead of treating the
		// motion as a straight line.
		Logger.recordOutput("Drive/RequestedChassisSpeeds", chassisSpeeds);
		ChassisSpeeds discretized = ChassisSpeeds.discretize(chassisSpeeds, DriveConstants.kLoopPeriodSeconds);
		Logger.recordOutput("Drive/DiscretizedChassisSpeeds", discretized);
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(discretized));
	}

	/**
	 * Caps the requested speeds so no wheel is asked to exceed the robot's top
	 * speed, then lets each module optimize its own steering angle. If any module
	 * is unhealthy or has a bad reading, the whole drivetrain stays stopped:
	 * three-wheel driving is not validated. A gyro fault only forces robot-oriented
	 * fallback, handled by the teleop command.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		if (!isDrivetrainHealthy()) {
			stopModules();
			Logger.recordOutput("Drive/DriveBlockedReason", blockedReason());
			return;
		}
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setDesiredState(desiredStates[i]);
		}
		Logger.recordOutput("Drive/Setpoints", desiredStates);
	}

	/** Holds the wheels in an X. Bound to the A button in RobotContainer. */
	public void lockWheels() {
		setModuleStates(DriveConstants.crossModuleStates());
	}

	/**
	 * Stops drive and steering. Used when the driver command ends or a fault
	 * occurs.
	 */
	public void stopModules() {
		for (SwerveModule module : modules) {
			module.stop();
		}
	}

	/** Current estimated position on the field. */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/** Current heading; field-oriented driving uses this. */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * True only when every module has trusted data. A single invalid encoder frame
	 * blocks driving immediately; the module-level debounce controls the fault
	 * report, not the safety stop.
	 */
	public boolean isDrivetrainHealthy() {
		for (SwerveModule module : modules) {
			if (!module.isHealthy() || !module.hasValidReading()) {
				return false;
			}
		}
		return true;
	}

	/** Debounced gyro health, for reporting. */
	public boolean isGyroHealthy() {
		return gyroHealthy;
	}

	/** Field-oriented driving needs a currently valid gyro reading. */
	public boolean isFieldOrientedAvailable() {
		return gyroHealthy && gyroReadingValid;
	}

	/** Empty when healthy; otherwise the active module/gyro faults. */
	public String getFaultSummary() {
		return faultSummary;
	}

	/**
	 * Sets the field position, for example at the start of an autonomous. The drive
	 * encoders are re-zeroed first so the pose estimator and the wheels agree after
	 * the reset.
	 */
	public void resetPose(Pose2d pose) {
		for (SwerveModule module : modules) {
			module.resetDrivePosition();
		}
		poseEstimator.resetPosition(new Rotation2d(effectiveYawRad()), getModulePositions(), pose);
		// Reject any camera frame captured before this reset.
		double now = Timer.getFPGATimestamp();
		lastAcceptedVisionTimestampSeconds = now;
		poseHistory.reset(now, getPose());
	}

	/**
	 * Makes the robot's current direction the new field-forward heading. The gyro,
	 * pose estimator, and field-oriented driving all use the same zero afterward.
	 * The field position and drive encoders are left alone.
	 */
	public void resetHeading() {
		gyroIO.resetYaw();
		gyroIO.updateInputs(gyroInputs);
		updateGyroHealth();
		Pose2d currentPose = getPose();
		poseEstimator.resetPosition(new Rotation2d(effectiveYawRad()), getModulePositions(),
				new Pose2d(currentPose.getTranslation(), new Rotation2d()));
		double now = Timer.getFPGATimestamp();
		lastAcceptedVisionTimestampSeconds = now;
		poseHistory.reset(now, getPose());
		headingResetCount++;
		Logger.recordOutput("Drive/HeadingResetCount", headingResetCount);
	}

	/**
	 * Feeds a timestamped camera pose into the estimator after checking that it is
	 * fresh, ordered, sane, on the field, and close enough to the pose the robot
	 * actually had at the measurement time. Accepted and rejected measurements are
	 * logged with the reason.
	 *
	 * <p>
	 * Call this from the robot main loop only. The pose estimator is not
	 * thread-safe; a camera callback thread must queue its measurement and let the
	 * main loop submit it.
	 *
	 * @param visionPose
	 *            field-relative camera pose
	 * @param timestampSeconds
	 *            capture time on the same clock as Timer
	 * @param stdDevs
	 *            x, y, and heading standard deviations
	 * @param tagCount
	 *            number of tags used by the camera
	 * @param averageTagDistanceMeters
	 *            average distance to those tags
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs, int tagCount,
			double averageTagDistanceMeters) {
		double now = Timer.getFPGATimestamp();
		double age = now - timestampSeconds;
		Pose2d currentPose = getPose();
		Pose2d historicalPose = poseHistory.get(timestampSeconds);
		double translationInnovation = visionPose == null || historicalPose == null
				? Double.NaN
				: historicalPose.getTranslation().getDistance(visionPose.getTranslation());
		double rotationInnovation = visionPose == null || historicalPose == null
				? Double.NaN
				: Math.abs(MathUtil.inputModulus(
						historicalPose.getRotation().getRadians() - visionPose.getRotation().getRadians(), -Math.PI,
						Math.PI));
		double currentTranslationInnovation = visionPose == null
				? Double.NaN
				: currentPose.getTranslation().getDistance(visionPose.getTranslation());
		double currentRotationInnovation = visionPose == null
				? Double.NaN
				: Math.abs(MathUtil.inputModulus(
						currentPose.getRotation().getRadians() - visionPose.getRotation().getRadians(), -Math.PI,
						Math.PI));

		String rejectionReason = null;
		if (visionPose == null) {
			rejectionReason = "null pose";
		} else if (!isFinite(visionPose)) {
			rejectionReason = "invalid pose";
		} else if (!Double.isFinite(timestampSeconds) || timestampSeconds <= 0.0) {
			rejectionReason = "invalid timestamp";
		} else if (age < -0.01) {
			rejectionReason = "future timestamp";
		} else if (age > VisionConstants.kMaxAgeSeconds) {
			rejectionReason = "stale";
		} else if (timestampSeconds < lastAcceptedVisionTimestampSeconds) {
			rejectionReason = "out of order";
		} else if (tagCount < VisionConstants.kMinTags) {
			rejectionReason = "too few tags";
		} else if (!isValidStdDevs(stdDevs)) {
			rejectionReason = "invalid covariance";
		} else if (visionPose.getX() < 0.0 || visionPose.getX() > VisionConstants.kFieldLengthMeters
				|| visionPose.getY() < 0.0 || visionPose.getY() > VisionConstants.kFieldWidthMeters) {
			rejectionReason = "outside field";
		} else if (historicalPose == null) {
			rejectionReason = "no pose history";
		} else if (translationInnovation > VisionConstants.kMaxTranslationInnovationMeters) {
			rejectionReason = "translation jump";
		} else if (rotationInnovation > VisionConstants.kMaxRotationInnovationRadians) {
			rejectionReason = "rotation jump";
		}

		if (rejectionReason == null) {
			poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
			lastAcceptedVisionTimestampSeconds = timestampSeconds;
			acceptedVisionCount++;
		} else {
			rejectedVisionCount++;
		}

		logVision(visionPose, historicalPose, timestampSeconds, stdDevs, tagCount, averageTagDistanceMeters, age,
				translationInnovation, rotationInnovation, currentTranslationInnovation, currentRotationInnovation,
				rejectionReason);
		Logger.recordOutput("Vision/AcceptedCount", acceptedVisionCount);
		Logger.recordOutput("Vision/RejectedCount", rejectedVisionCount);
	}

	private static boolean isFinite(Pose2d pose) {
		return Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
				&& Double.isFinite(pose.getRotation().getRadians());
	}

	private static boolean isValidStdDevs(Matrix<N3, N1> stdDevs) {
		if (stdDevs == null || stdDevs.getNumRows() != 3 || stdDevs.getNumCols() != 1) {
			return false;
		}
		for (int row = 0; row < 3; row++) {
			double value = stdDevs.get(row, 0);
			if (!Double.isFinite(value) || value <= 0.0) {
				return false;
			}
		}
		return true;
	}

	private static void logVision(Pose2d visionPose, Pose2d historicalPose, double timestampSeconds,
			Matrix<N3, N1> stdDevs, int tagCount, double averageTagDistanceMeters, double ageSeconds,
			double translationInnovation, double rotationInnovation, double currentTranslationInnovation,
			double currentRotationInnovation, String rejectionReason) {
		Logger.recordOutput("Vision/Measurement", visionPose == null ? new Pose2d() : visionPose);
		Logger.recordOutput("Vision/HistoricalPose", historicalPose == null ? new Pose2d() : historicalPose);
		Logger.recordOutput("Vision/TimestampSeconds", timestampSeconds);
		Logger.recordOutput("Vision/AgeSeconds", ageSeconds);
		Logger.recordOutput("Vision/TagCount", tagCount);
		Logger.recordOutput("Vision/AverageTagDistanceMeters", averageTagDistanceMeters);
		Logger.recordOutput("Vision/TranslationInnovationMeters", translationInnovation);
		Logger.recordOutput("Vision/RotationInnovationRadians", rotationInnovation);
		Logger.recordOutput("Vision/CurrentTranslationInnovationMeters", currentTranslationInnovation);
		Logger.recordOutput("Vision/CurrentRotationInnovationRadians", currentRotationInnovation);
		double[] stdDevValues;
		if (stdDevs == null || stdDevs.getNumRows() != 3 || stdDevs.getNumCols() != 1) {
			stdDevValues = new double[]{Double.NaN, Double.NaN, Double.NaN};
		} else {
			stdDevValues = new double[]{stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)};
		}
		Logger.recordOutput("Vision/StdDevs", stdDevValues);
		Logger.recordOutput("Vision/Accepted", rejectionReason == null);
		Logger.recordOutput("Vision/RejectedReason", rejectionReason == null ? "" : rejectionReason);
	}

	private void updateGyroHealth() {
		boolean valid = gyroInputs.connected && Double.isFinite(gyroInputs.yawRad);
		if (valid) {
			lastValidYawRad = gyroInputs.yawRad;
			hasValidYaw = true;
			gyroReadingValid = true;
			gyroInvalidFrames = 0;
			if (!gyroHealthy) {
				gyroRecoveryFrames++;
				if (gyroRecoveryFrames >= HealthConstants.kGyroRecoveryFrames) {
					gyroHealthy = true;
					gyroRecoveryFrames = 0;
					gyroRecoveryCount++;
				}
			}
			return;
		}

		gyroReadingValid = false;
		gyroRecoveryFrames = 0;
		gyroInvalidFrames++;
		if (gyroHealthy && gyroInvalidFrames >= HealthConstants.kGyroFaultDebounceFrames) {
			gyroHealthy = false;
		}
	}

	/** Last good yaw while the gyro is faulted; never throws away heading. */
	private double effectiveYawRad() {
		return gyroReadingValid ? gyroInputs.yawRad : (hasValidYaw ? lastValidYawRad : 0.0);
	}

	private void updateFaultReporting() {
		StringBuilder summary = new StringBuilder();
		for (int i = 0; i < modules.length; i++) {
			SwerveModule module = modules[i];
			if (!module.isHealthy()) {
				appendFault(summary, "module " + i + ": " + module.getFaultReason());
			}
		}
		if (!gyroHealthy) {
			appendFault(summary, "gyro: disconnected or stale");
		}

		String newSummary = summary.toString();
		boolean active = !newSummary.isEmpty();
		if (active && !faultActive) {
			DriverStation.reportError("Drivetrain fault: " + newSummary + ". " + blockedReason(), false);
			faultCount++;
		} else if (!active && faultActive) {
			DriverStation.reportWarning("Drivetrain recovered: " + faultSummary, false);
			recoveryCount++;
		}
		faultActive = active;
		faultSummary = newSummary;

		Logger.recordOutput("Drive/HardwareMode", Constants.kHardware.name());
		Logger.recordOutput("Drive/FaultActive", faultActive);
		Logger.recordOutput("Drive/FaultReason", faultSummary);
		Logger.recordOutput("Drive/FaultCount", faultCount);
		Logger.recordOutput("Drive/RecoveryCount", recoveryCount);
		Logger.recordOutput("Drive/GyroHealthy", gyroHealthy);
		Logger.recordOutput("Drive/GyroReadingValid", gyroReadingValid);
		Logger.recordOutput("Drive/GyroFallbackActive", !isFieldOrientedAvailable());
		Logger.recordOutput("Drive/GyroRecoveryCount", gyroRecoveryCount);
	}

	/** Human-readable reason driving is blocked, or an empty string. */
	private String blockedReason() {
		if (gyroHealthy && gyroReadingValid && isDrivetrainHealthy()) {
			return "";
		}
		StringBuilder reason = new StringBuilder();
		for (int i = 0; i < modules.length; i++) {
			SwerveModule module = modules[i];
			if (!module.isHealthy()) {
				appendFault(reason, "module " + i + " faulted: " + module.getFaultReason());
			} else if (!module.hasValidReading()) {
				appendFault(reason, "module " + i + " bad reading: " + module.getInvalidReason());
			}
		}
		if (!isDrivetrainHealthy()) {
			appendFault(reason, "driving disabled until every module is healthy");
		}
		if (!gyroHealthy || !gyroReadingValid) {
			appendFault(reason, "field heading unavailable; robot-oriented fallback");
		}
		return reason.toString();
	}

	private static void appendFault(StringBuilder builder, String fault) {
		if (builder.length() > 0) {
			builder.append("; ");
		}
		builder.append(fault);
	}

	/** Simulation only: the rate the robot is actually rotating. */
	private double getMeasuredAngularVelocity() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
	}

	/** Test-only access to one module. */
	SwerveModule getModuleForTesting(int index) {
		return modules[index];
	}

	/** Test-only vision counters. */
	int getAcceptedVisionCountForTesting() {
		return acceptedVisionCount;
	}

	int getRejectedVisionCountForTesting() {
		return rejectedVisionCount;
	}
}
