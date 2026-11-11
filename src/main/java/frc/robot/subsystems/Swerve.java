package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Hardware;
import frc.robot.Constants.SwerveModuleConfig;
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

	public Swerve() {
		// Pick the REV or CTRE wiring set from Constants.kHardware.
		SwerveModuleConfig[] configs = Constants.kHardware == Hardware.CTRE_MK5N
				? DriveConstants.kCtreModuleConfigs
				: DriveConstants.kRevModuleConfigs;

		// Build one module per wheel. Simulation uses the same SwerveModule logic
		// but swaps the hardware layer for SwerveModuleIOSim.
		for (int i = 0; i < modules.length; i++) {
			SwerveModuleIO io;
			if (RobotBase.isSimulation()) {
				io = new SwerveModuleIOSim();
			} else if (Constants.kHardware == Hardware.CTRE_MK5N) {
				io = new SwerveModuleIOTalonFX(configs[i]);
			} else {
				io = new SwerveModuleIOSparkMax(configs[i]);
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
		// 1) Read every module and run its steering loop.
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
		setModuleStates(DriveConstants.kCrossModuleStates);
	}

	/** Stops drive and steering. Used when the driver command ends. */
	public void stopModules() {
		for (SwerveModule module : modules) {
			module.stop();
		}
	}

	/** Pit helper: re-zero the drive encoders after changing a wheel. */
	public void resetDrivePositions() {
		for (SwerveModule module : modules) {
			module.resetDrivePosition();
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

	/** Sets the field position, for example at the start of an autonomous. */
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(new Rotation2d(gyroInputs.yawRad), getModulePositions(), pose);
	}

	/** Future vision hook: feed a camera pose into the estimator. */
	public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
	}

	/** Simulation only: the rate the robot is actually rotating. */
	private double getMeasuredAngularVelocity() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
	}
}
