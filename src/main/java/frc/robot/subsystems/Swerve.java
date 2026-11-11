package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Hardware;
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
	/** Used only to report the real loop time in AdvantageScope. */
	private double lastTimestampSeconds = Timer.getFPGATimestamp();
	/** Vision measurements older than the last accepted one are ignored. */
	private double lastAcceptedVisionTimestampSeconds = -1.0;
	private int acceptedVisionCount = 0;
	private int rejectedVisionCount = 0;

	public Swerve() {
		// Pick the REV or CTRE wiring set from Constants.kHardware.
		List<SwerveModuleConfig> configs = Constants.kHardware == Hardware.CTRE_MK5N
				? DriveConstants.kCtreModuleConfigs
				: DriveConstants.kRevModuleConfigs;

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
	}

	/**
	 * Reads the hardware once so the starting pose uses the real wheel angles
	 * instead of zeros. Also tells each module to hold its current angle.
	 */
	private SwerveDrivePoseEstimator createPoseEstimator() {
		for (SwerveModule module : modules) {
			module.refreshInputs();
			module.holdCurrentAngle();
		}
		gyroIO.updateInputs(gyroInputs);
		return new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, new Rotation2d(gyroInputs.yawRad),
				getModulePositions(), new Pose2d());
	}

	@Override
	public void periodic() {
		// 1) Read every module and run its drive and steering loops.
		for (SwerveModule module : modules) {
			module.periodic();
		}

		// 2) Read the gyro.
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);

		// 3) Update the pose from wheel motion and gyro heading.
		poseEstimator.update(new Rotation2d(gyroInputs.yawRad), getModulePositions());

		// 4) Log what the robot actually did this cycle.
		double now = Timer.getFPGATimestamp();
		Logger.recordOutput("Drive/LoopTimeSeconds", now - lastTimestampSeconds);
		lastTimestampSeconds = now;
		Logger.recordOutput("Drive/BatteryVoltage", RobotController.getBatteryVoltage());
		Logger.recordOutput("Drive/MeasuredChassisSpeeds", getRobotRelativeSpeeds());
		Logger.recordOutput("Drive/Pose", getPose());
		Logger.recordOutput("Drive/Heading", getHeading());
		Logger.recordOutput("Drive/MeasuredStates", getModuleStates());
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
	 * speed, then lets each module optimize its own steering angle.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
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

	/** Stops drive and steering. Used when the driver command ends. */
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
	 * Sets the field position, for example at the start of an autonomous. The drive
	 * encoders are re-zeroed first so the pose estimator and the wheels agree after
	 * the reset.
	 */
	public void resetPose(Pose2d pose) {
		for (SwerveModule module : modules) {
			module.resetDrivePosition();
		}
		poseEstimator.resetPosition(new Rotation2d(gyroInputs.yawRad), getModulePositions(), pose);
		// Reject any camera frame captured before this reset.
		lastAcceptedVisionTimestampSeconds = Timer.getFPGATimestamp();
	}

	/**
	 * Convenience overload with the default covariance: translation is trusted,
	 * heading is effectively ignored.
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
		addVisionMeasurement(visionPose, timestampSeconds,
				VecBuilder.fill(VisionConstants.kDefaultTranslationStdDevMeters,
						VisionConstants.kDefaultTranslationStdDevMeters, VisionConstants.kDefaultRotationStdDevRadians),
				1, 0.0);
	}

	/**
	 * Feeds a timestamped camera pose into the estimator after checking that it is
	 * fresh, ordered, sane, and close enough to the current pose. Accepted and
	 * rejected measurements are logged with the reason.
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
		double translationInnovation = visionPose == null
				? Double.NaN
				: currentPose.getTranslation().getDistance(visionPose.getTranslation());
		double rotationInnovation = visionPose == null
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

		logVision(visionPose, timestampSeconds, stdDevs, tagCount, averageTagDistanceMeters, age, translationInnovation,
				rotationInnovation, rejectionReason);
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

	private static void logVision(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs, int tagCount,
			double averageTagDistanceMeters, double ageSeconds, double translationInnovation, double rotationInnovation,
			String rejectionReason) {
		Logger.recordOutput("Vision/Measurement", visionPose == null ? new Pose2d() : visionPose);
		Logger.recordOutput("Vision/TimestampSeconds", timestampSeconds);
		Logger.recordOutput("Vision/AgeSeconds", ageSeconds);
		Logger.recordOutput("Vision/TagCount", tagCount);
		Logger.recordOutput("Vision/AverageTagDistanceMeters", averageTagDistanceMeters);
		Logger.recordOutput("Vision/TranslationInnovationMeters", translationInnovation);
		Logger.recordOutput("Vision/RotationInnovationRadians", rotationInnovation);
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
