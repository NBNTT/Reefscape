package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CtreConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HealthConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.Autos;
import frc.robot.commands.drivetrain.ResetHeading;
import frc.robot.commands.drivetrain.SwerveTeleOp;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

/**
 * Logic tests that inject fake hardware. These cover module order, gyro signs,
 * pose resets, steering optimization, vendor steering, stop behavior, command
 * interruption, and vision gating without touching a real robot.
 */
class SwerveControlTest {
	@Test
	void moduleOrderAndKinematicsArePreserved() {
		FakeModuleIO[] ios = createFakeModules();
		for (int i = 0; i < ios.length; i++) {
			ios[i].drivePositionMeters = i + 1.0;
			ios[i].driveVelocityMetersPerSec = 1.0;
		}
		Swerve swerve = new Swerve(ios, new FakeGyroIO());

		assertEquals(1.0, swerve.getModulePositions()[0].distanceMeters, 1e-9);
		assertEquals(2.0, swerve.getModulePositions()[1].distanceMeters, 1e-9);
		assertEquals(3.0, swerve.getModulePositions()[2].distanceMeters, 1e-9);
		assertEquals(4.0, swerve.getModulePositions()[3].distanceMeters, 1e-9);
		assertEquals(1.0, swerve.getRobotRelativeSpeeds().vxMetersPerSecond, 1e-9);

		// Feed distinct speeds in FL, FR, BL, BR order and verify they stay put.
		SwerveModuleState[] requested = {new SwerveModuleState(1.0, new Rotation2d()),
				new SwerveModuleState(2.0, new Rotation2d()), new SwerveModuleState(3.0, new Rotation2d()),
				new SwerveModuleState(4.0, new Rotation2d())};
		swerve.setModuleStates(requested);
		for (int i = 0; i < requested.length; i++) {
			assertEquals(requested[i].speedMetersPerSecond,
					swerve.getModuleForTesting(i).getSetpointForTesting().speedMetersPerSecond, 1e-9,
					"Module order mismatch at index " + i);
		}
	}

	@Test
	void gyroSignIsPositiveCounterClockwise() {
		GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();
		GyroIOSim gyro = new GyroIOSim(() -> 1.0);
		for (int i = 0; i < 50; i++) {
			gyro.updateInputs(inputs);
		}
		assertEquals(1.0, inputs.yawRad, 1e-9);

		FakeGyroIO fakeGyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), fakeGyro);
		// Change the heading after construction so the estimator sees a delta.
		fakeGyro.yawRad = 0.5;
		swerve.periodic();
		assertEquals(0.5, swerve.getHeading().getRadians(), 1e-9);
	}

	@Test
	void resetPoseAlsoResetsDriveEncoders() {
		FakeModuleIO[] ios = createFakeModules();
		for (int i = 0; i < ios.length; i++) {
			ios[i].drivePositionMeters = i + 1.0;
		}
		FakeGyroIO gyro = new FakeGyroIO();
		gyro.yawRad = 0.2;
		Swerve swerve = new Swerve(ios, gyro);

		swerve.resetPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)));

		assertEquals(1.0, swerve.getPose().getX(), 1e-9);
		assertEquals(2.0, swerve.getPose().getY(), 1e-9);
		assertEquals(90.0, swerve.getPose().getRotation().getDegrees(), 1e-9);
		for (FakeModuleIO io : ios) {
			assertEquals(0.0, io.drivePositionMeters, 1e-9);
			assertEquals(1, io.resetCount);
		}
	}

	@Test
	void steeringOptimizesToShortestPath() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		module.setDesiredState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(180.0)));

		SwerveModuleState setpoint = module.getSetpointForTesting();
		assertEquals(0.0, setpoint.angle.getDegrees(), 1e-9);
		assertTrue(setpoint.speedMetersPerSecond < 0.0);
	}

	@Test
	void cosineCompensationReducesDriveWhenMisaligned() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		module.setDesiredState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(90.0)));

		SwerveModuleState setpoint = module.getSetpointForTesting();
		assertEquals(90.0, setpoint.angle.getDegrees(), 1e-9);
		assertEquals(0.0, setpoint.speedMetersPerSecond, 1e-6);
	}

	@Test
	void initializationDoesNotDriveOrSteer() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		module.periodic();

		assertEquals(0.0, io.driveVoltage, 1e-9);
		assertEquals(0.0, io.turnOutput, 1e-9);
	}

	@Test
	void stopZerosDriveAndHoldsAngle() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		module.setDesiredState(new SwerveModuleState(1.0, new Rotation2d()));

		module.stop();

		assertEquals(0.0, io.driveVoltage, 1e-9);
		assertEquals(0.0, io.turnOutput, 1e-9);
		assertEquals(0.0, module.getSetpointForTesting().speedMetersPerSecond, 1e-9);
	}

	@Test
	void vendorSteeringUsesPositionControl() {
		FakeModuleIO io = new FakeModuleIO();
		io.useTurnPositionControl = true;
		SwerveModule module = new SwerveModule(0, io);
		module.setDesiredState(new SwerveModuleState(1.0, new Rotation2d()));

		assertEquals(0.0, io.turnPositionCommand, 1e-9);
		assertEquals(0.0, io.turnOutput, 1e-9);
	}

	@Test
	void disabledModeStopsAndDoesNotDrive() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());
		double[] rumble = {1.0};

		SwerveTeleOp command = new SwerveTeleOp(swerve, () -> 1.0, () -> 0.0, () -> 0.0, () -> false, () -> false,
				() -> false, value -> rumble[0] = value);
		command.initialize();
		command.execute();

		assertEquals(0.0, rumble[0], 1e-9);
		assertFalse(command.isHeadingHoldActiveForTesting());
		assertEquals(0.0, command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond, 1e-9);
		for (FakeModuleIO io : ios) {
			assertEquals(0.0, io.driveVoltage, 1e-9);
			assertEquals(0.0, io.turnOutput, 1e-9);
		}
	}

	@Test
	void startResetZerosHeading() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		gyro.yawRad = 0.7;
		swerve.periodic();
		assertEquals(0.7, swerve.getHeading().getRadians(), 1e-9);

		swerve.resetHeading();

		assertEquals(0.0, swerve.getHeading().getRadians(), 1e-9);
		assertEquals(0.0, gyro.yawRad, 1e-9);
		assertEquals(1, gyro.resetCount);
	}

	@Test
	void headingHoldCapturesAfterRelease() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		double[] turn = {0.5};
		SwerveTeleOp command = createDriveCommand(swerve, () -> 0.0, () -> turn[0], () -> false, () -> true);
		command.initialize();

		for (int i = 0; i < 5; i++) {
			command.execute();
		}
		assertFalse(command.isHeadingHoldActiveForTesting());

		turn[0] = 0.0;
		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());

		gyro.yawRad = 0.2;
		swerve.periodic();
		for (int i = 0; i < 30; i++) {
			command.execute();
		}
		assertTrue(command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond < 0.0);
	}

	@Test
	void headingHoldHoldsWhileTranslating() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		double[] forward = {0.0};
		SwerveTeleOp command = createDriveCommand(swerve, () -> forward[0], () -> 0.0, () -> false, () -> true);
		command.initialize();
		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());

		forward[0] = 0.5;
		for (int i = 0; i < 5; i++) {
			command.execute();
		}
		assertEquals(0.0, command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond, 1e-9);
	}

	@Test
	void manualRotationOverridesHeadingHold() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		double[] turn = {0.0};
		SwerveTeleOp command = createDriveCommand(swerve, () -> 0.0, () -> turn[0], () -> false, () -> true);
		command.initialize();
		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());

		turn[0] = 0.5;
		command.execute();
		assertFalse(command.isHeadingHoldActiveForTesting());
		assertTrue(command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond > 0.0);
	}

	@Test
	void headingHoldWrapsNearPi() {
		FakeGyroIO gyro = new FakeGyroIO();
		gyro.yawRad = 3.1;
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		swerve.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(3.1)));
		SwerveTeleOp command = createDriveCommand(swerve, () -> 0.0, () -> 0.0, () -> false, () -> true);
		command.initialize();
		command.execute();
		assertEquals(3.1, swerve.getHeading().getRadians(), 1e-9);

		gyro.yawRad = -3.1;
		swerve.periodic();
		for (int i = 0; i < 10; i++) {
			command.execute();
		}
		assertTrue(command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond < 0.0);
	}

	@Test
	void resetHeadingClearsHeadingHold() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		SwerveTeleOp command = createDriveCommand(swerve, () -> 0.0, () -> 0.0, () -> false, () -> true);
		command.initialize();
		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());

		gyro.yawRad = 0.5;
		swerve.periodic();
		command.execute();
		swerve.resetHeading();
		command.resetHeadingHold();
		for (int i = 0; i < 10; i++) {
			command.execute();
		}

		assertTrue(command.isHeadingHoldActiveForTesting());
		assertEquals(0.0, swerve.getHeading().getRadians(), 1e-9);
		assertEquals(0.0, command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond, 1e-6);
	}

	@Test
	void robotOrientedDisablesHeadingHold() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		SwerveTeleOp command = createDriveCommand(swerve, () -> 0.0, () -> 0.0, () -> true, () -> true);
		command.initialize();
		command.execute();

		assertFalse(command.isHeadingHoldActiveForTesting());
		assertEquals(0.0, command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond, 1e-9);
	}

	@Test
	void fullStickReachesConfiguredMaximums() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		SwerveTeleOp command = createDriveCommand(swerve, () -> 1.0, () -> 1.0, () -> false, () -> true);
		command.initialize();

		for (int i = 0; i < 30; i++) {
			command.execute();
		}

		assertEquals(DriveConstants.kMaxSpeedMetersPerSecond,
				command.getLastChassisSpeedsForTesting().vxMetersPerSecond, 1e-6);
		assertEquals(DriveConstants.kMaxAngularSpeedRadiansPerSecond,
				command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond, 1e-6);
	}

	@Test
	void slowModeScalesAndRumbles() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		double[] rumble = {0.0};
		SwerveTeleOp command = new SwerveTeleOp(swerve, () -> 1.0, () -> 0.0, () -> 0.0, () -> true, () -> false,
				() -> true, value -> rumble[0] = value);
		command.initialize();

		for (int i = 0; i < 30; i++) {
			command.execute();
		}

		assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * OIConstants.kSlowModeFactor,
				command.getLastChassisSpeedsForTesting().vxMetersPerSecond, 1e-6);
		assertEquals(OIConstants.kRumble, rumble[0], 1e-9);
	}

	@Test
	void autonomousDriveForwardRunsAndStops() {
		Swerve swerve = createSimSwerve();
		CommandScheduler scheduler = CommandScheduler.getInstance();
		scheduler.cancelAll();
		// The test harness has no enabled Driver Station, so let the auto run.
		Command auto = Autos.driveForward(swerve, 1.0, 0.1).ignoringDisable(true);

		auto.schedule();
		for (int i = 0; i < 30; i++) {
			Timer.delay(0.02);
			scheduler.run();
			// The test drives the subsystem clock explicitly as well as the
			// scheduler clock so the pose integrates deterministically.
			swerve.periodic();
		}

		assertTrue(swerve.getPose().getX() > 0.01,
				"Pose was " + swerve.getPose() + ", scheduled=" + auto.isScheduled());
		assertFalse(auto.isScheduled());
		scheduler.cancelAll();
	}

	@Test
	void pureStrafeStillReachesMaximum() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		SwerveTeleOp command = new SwerveTeleOp(swerve, () -> 0.0, () -> 1.0, () -> 0.0, () -> false, () -> false,
				() -> true, value -> {
				});
		command.initialize();

		for (int i = 0; i < 30; i++) {
			command.execute();
		}

		assertEquals(DriveConstants.kMaxSpeedMetersPerSecond,
				command.getLastChassisSpeedsForTesting().vyMetersPerSecond, 1e-6);
	}

	@Test
	void lowStickHasFinerControl() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		SwerveTeleOp command = createDriveCommand(swerve, () -> 0.5, () -> 0.0, () -> false, () -> true);
		command.initialize();

		for (int i = 0; i < 30; i++) {
			command.execute();
		}

		double expected = Math.pow((0.5 - OIConstants.kDeadband) / (1.0 - OIConstants.kDeadband), 2)
				* DriveConstants.kMaxSpeedMetersPerSecond;
		assertEquals(expected, command.getLastChassisSpeedsForTesting().vxMetersPerSecond, 1e-6);
		assertTrue(command.getLastChassisSpeedsForTesting().vxMetersPerSecond < 0.5
				* DriveConstants.kMaxSpeedMetersPerSecond);
	}

	@Test
	void teleopInterruptionStopsAndClearsRumble() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());
		double[] rumble = {1.0};

		SwerveTeleOp command = new SwerveTeleOp(swerve, () -> 1.0, () -> 0.0, () -> 0.0, () -> false, () -> true,
				() -> true, value -> rumble[0] = value);
		command.initialize();
		command.execute();
		command.end(true);

		assertEquals(0.0, rumble[0], 1e-9);
		assertFalse(command.isHeadingHoldActiveForTesting());
		assertEquals(0.0, command.getLastChassisSpeedsForTesting().omegaRadiansPerSecond, 1e-9);
		for (FakeModuleIO io : ios) {
			assertEquals(0.0, io.driveVoltage, 1e-9);
			assertEquals(0.0, io.turnOutput, 1e-9);
		}
	}

	@Test
	void visionAcceptsGoodMeasurementsAndRejectsBadOnes() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());
		double now = Timer.getFPGATimestamp();

		swerve.addVisionMeasurement(new Pose2d(0.1, 0.0, new Rotation2d()), now, VecBuilder.fill(0.1, 0.1, 0.1), 2,
				1.0);
		assertEquals(1, swerve.getAcceptedVisionCountForTesting());

		// A five-meter jump is not believable.
		swerve.addVisionMeasurement(new Pose2d(5.0, 0.0, new Rotation2d()), now + 0.001, VecBuilder.fill(0.1, 0.1, 0.1),
				2, 1.0);
		assertEquals(1, swerve.getRejectedVisionCountForTesting());

		// A one-second-old frame is stale.
		swerve.addVisionMeasurement(new Pose2d(0.1, 0.0, new Rotation2d()), now - 1.0, VecBuilder.fill(0.1, 0.1, 0.1),
				2, 1.0);
		// A one-second future frame is not valid.
		swerve.addVisionMeasurement(new Pose2d(0.1, 0.0, new Rotation2d()), now + 1.0, VecBuilder.fill(0.1, 0.1, 0.1),
				2, 1.0);
		// A pose outside the field is not valid.
		swerve.addVisionMeasurement(new Pose2d(-0.5, 0.0, new Rotation2d()), now, VecBuilder.fill(0.1, 0.1, 0.1), 2,
				1.0);
		assertEquals(4, swerve.getRejectedVisionCountForTesting());
	}

	@Test
	void visionRejectsOutOfOrderLowTagAndBadCovariance() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());
		double now = Timer.getFPGATimestamp();

		// First measurement establishes the last-accepted timestamp.
		swerve.addVisionMeasurement(new Pose2d(0.0, 0.0, new Rotation2d()), now, VecBuilder.fill(0.1, 0.1, 0.1), 2,
				1.0);
		// Older than the accepted frame.
		swerve.addVisionMeasurement(new Pose2d(0.0, 0.0, new Rotation2d()), now - 0.1, VecBuilder.fill(0.1, 0.1, 0.1),
				2, 1.0);
		// Too few tags.
		swerve.addVisionMeasurement(new Pose2d(0.0, 0.0, new Rotation2d()), now + 0.005, VecBuilder.fill(0.1, 0.1, 0.1),
				0, 1.0);
		// Zero covariance is invalid.
		swerve.addVisionMeasurement(new Pose2d(0.0, 0.0, new Rotation2d()), now + 0.006, VecBuilder.fill(0.0, 0.1, 0.1),
				2, 1.0);

		assertEquals(1, swerve.getAcceptedVisionCountForTesting());
		assertEquals(3, swerve.getRejectedVisionCountForTesting());
	}

	@Test
	void fieldAndRobotOrientedDrivingMatchAtDifferentHeadings() {
		assertDrivingMatches(0.0, false);
		assertDrivingMatches(0.5, false);
		assertDrivingMatches(0.5, true);
	}

	@Test
	void periodicNeverResetsDriveEncoders() {
		FakeModuleIO[] ios = createFakeModules();
		for (int i = 0; i < ios.length; i++) {
			ios[i].drivePositionMeters = 1.5 + i;
		}
		Swerve swerve = new Swerve(ios, new FakeGyroIO());
		for (int i = 0; i < 50; i++) {
			swerve.periodic();
		}
		for (int i = 0; i < ios.length; i++) {
			assertEquals(0, ios[i].resetCount);
			assertEquals(1.5 + i, ios[i].drivePositionMeters, 1e-9);
		}
	}

	@Test
	void angleWrappingUsesShortestPath() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		module.setDesiredState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(359.0)));

		SwerveModuleState setpoint = module.getSetpointForTesting();
		assertEquals(-1.0, setpoint.angle.getDegrees(), 1e-9);
		assertTrue(setpoint.speedMetersPerSecond > 0.0);
		assertTrue(io.turnOutput < 0.0);
	}

	@Test
	void driveVoltageIsClampedAndZeroAtZeroSpeed() {
		FakeModuleIO fastIO = new FakeModuleIO();
		SwerveModule fastModule = new SwerveModule(0, fastIO);
		fastModule.setDesiredState(new SwerveModuleState(100.0, new Rotation2d()));
		assertEquals(12.0, fastIO.driveVoltage, 1e-9);

		FakeModuleIO stoppedIO = new FakeModuleIO();
		SwerveModule stoppedModule = new SwerveModule(0, stoppedIO);
		stoppedModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d()));
		assertEquals(0.0, stoppedIO.driveVoltage, 1e-9);
	}

	@Test
	void autonomousFoundationIsWiredToTheDrivetrain() {
		Swerve swerve = new Swerve(createFakeModules(), new FakeGyroIO());
		Command doNothing = Autos.doNothing();
		Command driveForward = Autos.driveForward(swerve, 1.0, 0.1);

		assertNotNull(doNothing);
		assertTrue(driveForward.getRequirements().contains(swerve));
	}

	@Test
	void measuredVelocityControlDoesNotUseOpenLoopPercent() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		module.setDesiredState(new SwerveModuleState(1.0, new Rotation2d()));

		// Feedforward (kV * 1 m/s) plus feedback (kP * 1 m/s error) in volts.
		assertEquals(DriveConstants.kDriveKvVoltsPerMps + DriveConstants.kDriveKp, io.driveVoltage, 1e-6);
		assertTrue(io.driveVoltage <= 12.0);
	}

	@Test
	void chassisSpeedsGoThroughDiscretize() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());
		// A pure rotation exercises the discretize path without changing module order.
		swerve.setModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
		for (int i = 0; i < ios.length; i++) {
			assertTrue(Double.isFinite(swerve.getModuleForTesting(i).getSetpointForTesting().angle.getRadians()));
		}
	}

	@Test
	void absoluteEncoderDisconnectStopsModuleAndRecovers() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);
		assertTrue(module.isHealthy());

		io.absoluteEncoderConnected = false;
		module.periodic();
		assertFalse(module.hasValidReading());
		assertEquals("absolute encoder disconnected", module.getInvalidReason());

		module.setDesiredState(new SwerveModuleState(1.0, new Rotation2d()));
		assertEquals(0.0, io.driveVoltage, 1e-9);
		assertEquals(0.0, io.turnOutput, 1e-9);

		for (int i = 0; i < HealthConstants.kEncoderFaultDebounceFrames; i++) {
			module.periodic();
		}
		assertFalse(module.isHealthy());
		assertTrue(module.getFaultReason().contains("encoder"));

		io.absoluteEncoderConnected = true;
		for (int i = 0; i < HealthConstants.kEncoderRecoveryFrames; i++) {
			module.periodic();
		}
		assertTrue(module.isHealthy());
		assertTrue(module.getFaultReason().isEmpty());
	}

	@Test
	void nonFiniteAndJumpingAnglesAreRejected() {
		FakeModuleIO io = new FakeModuleIO();
		SwerveModule module = new SwerveModule(0, io);

		io.turnAbsolutePositionRad = Double.NaN;
		module.periodic();
		assertFalse(module.hasValidReading());
		assertEquals("non-finite absolute angle", module.getInvalidReason());
		assertEquals(0.0, module.getPosition().angle.getRadians(), 1e-9);

		// A 2 rad jump is not physically plausible in 20 ms.
		io.turnAbsolutePositionRad = 2.0;
		for (int i = 0; i < HealthConstants.kEncoderFaultDebounceFrames; i++) {
			module.periodic();
		}
		assertFalse(module.isHealthy());
		assertEquals(0.0, module.getPosition().angle.getRadians(), 1e-9);

		// Once the new angle stays stable, it is accepted and the module recovers.
		for (int i = 0; i < HealthConstants.kAngleJumpConfirmFrames + HealthConstants.kEncoderRecoveryFrames; i++) {
			module.periodic();
		}
		assertTrue(module.isHealthy());
		assertEquals(2.0, module.getPosition().angle.getRadians(), 1e-9);
	}

	@Test
	void moduleFaultDisablesTheWholeDrivetrain() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());

		ios[2].absoluteEncoderConnected = false;
		for (int i = 0; i < HealthConstants.kEncoderFaultDebounceFrames; i++) {
			swerve.periodic();
		}
		assertFalse(swerve.isDrivetrainHealthy());
		assertTrue(swerve.getFaultSummary().contains("module 2"));

		swerve.setModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));
		for (FakeModuleIO io : ios) {
			assertEquals(0.0, io.driveVoltage, 1e-9);
			assertEquals(0.0, io.turnOutput, 1e-9);
		}
	}

	@Test
	void gyroDisconnectFallsBackToRobotOrientedAndRecovers() {
		FakeGyroIO gyro = new FakeGyroIO();
		gyro.yawRad = 0.5;
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		swerve.periodic();
		assertTrue(swerve.isFieldOrientedAvailable());

		SwerveTeleOp command = createDriveCommand(swerve, () -> 1.0, () -> 0.0, () -> false, () -> true);
		command.initialize();
		for (int i = 0; i < 30; i++) {
			command.execute();
		}
		assertTrue(command.isHeadingHoldActiveForTesting());

		gyro.connected = false;
		for (int i = 0; i < HealthConstants.kGyroFaultDebounceFrames; i++) {
			swerve.periodic();
		}
		assertFalse(swerve.isGyroHealthy());
		assertFalse(swerve.isFieldOrientedAvailable());

		command.execute();
		assertFalse(command.isHeadingHoldActiveForTesting());
		assertEquals(DriveConstants.kMaxSpeedMetersPerSecond,
				command.getLastChassisSpeedsForTesting().vxMetersPerSecond, 1e-6);
		assertEquals(0.0, command.getLastChassisSpeedsForTesting().vyMetersPerSecond, 1e-6);

		gyro.connected = true;
		for (int i = 0; i < HealthConstants.kGyroRecoveryFrames; i++) {
			swerve.periodic();
		}
		assertTrue(swerve.isGyroHealthy());
		assertTrue(swerve.isFieldOrientedAvailable());

		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());
	}

	@Test
	void semanticRobotOrientedToggleTransitionsCleanly() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		AtomicBoolean robotOriented = new AtomicBoolean(false);
		SwerveTeleOp command = new SwerveTeleOp(swerve, () -> 0.0, () -> 0.0, () -> 0.0, () -> false,
				robotOriented::get, () -> true, value -> {
				});
		command.initialize();

		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());

		robotOriented.set(true);
		command.execute();
		assertFalse(command.isHeadingHoldActiveForTesting());

		robotOriented.set(false);
		command.execute();
		assertTrue(command.isHeadingHoldActiveForTesting());
	}

	@Test
	void delayedVisionUsesHistoricalPose() {
		FakeModuleIO[] ios = createFakeModules();
		Swerve swerve = new Swerve(ios, new FakeGyroIO());

		// Drive to about 2 m and record that pose in history.
		for (FakeModuleIO io : ios) {
			io.drivePositionMeters = 2.0;
		}
		swerve.periodic();
		double captureTime = Timer.getFPGATimestamp();
		assertEquals(2.0, swerve.getPose().getX(), 1e-6);

		// Move another 2 m. A frame captured at the old timestamp is now far from
		// the current pose but close to the historical one.
		for (FakeModuleIO io : ios) {
			io.drivePositionMeters = 4.0;
		}
		swerve.periodic();
		assertEquals(4.0, swerve.getPose().getX(), 1e-6);

		swerve.addVisionMeasurement(new Pose2d(2.1, 0.0, new Rotation2d()), captureTime, VecBuilder.fill(0.1, 0.1, 0.1),
				2, 1.0);

		assertEquals(1, swerve.getAcceptedVisionCountForTesting());
		assertEquals(0, swerve.getRejectedVisionCountForTesting());
	}

	@Test
	void resetHeadingCommandRunsWhileDisabled() {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		gyro.yawRad = 0.7;
		swerve.periodic();
		SwerveTeleOp drive = createDriveCommand(swerve, () -> 0.0, () -> 0.0, () -> false, () -> true);
		ResetHeading reset = new ResetHeading(swerve, drive);

		assertTrue(reset.runsWhenDisabled());
		CommandScheduler scheduler = CommandScheduler.getInstance();
		scheduler.cancelAll();
		reset.schedule();
		scheduler.run();

		assertEquals(0.0, swerve.getHeading().getRadians(), 1e-9);
		assertEquals(1, gyro.resetCount);
		scheduler.cancelAll();
	}

	@Test
	void ctreSteeringTargetWrapsAcrossPi() {
		assertEquals(1.05, SwerveModuleIOTalonFX.closestContinuousTarget(0.95, 0.05), 1e-9);
		assertEquals(0.51, SwerveModuleIOTalonFX.closestContinuousTarget(0.49, -0.49), 1e-9);
		assertEquals(-0.51, SwerveModuleIOTalonFX.closestContinuousTarget(-0.49, 0.49), 1e-9);
		assertEquals(0.49, SwerveModuleIOTalonFX.closestContinuousTarget(0.0, 0.49), 1e-9);
		assertEquals(-0.49, SwerveModuleIOTalonFX.closestContinuousTarget(0.0, -0.49), 1e-9);
	}

	@Test
	void ctreConfigurationRetriesTransientFailuresThenReports() {
		AtomicInteger attempts = new AtomicInteger();
		StatusCode recovered = SwerveModuleIOTalonFX.applyWithRetry(() -> {
			return attempts.incrementAndGet() < 2 ? StatusCode.EcuIsNotPresent : StatusCode.OK;
		}, "test device");
		assertTrue(recovered.isOK());
		assertEquals(2, attempts.get());

		attempts.set(0);
		StatusCode failed = SwerveModuleIOTalonFX.applyWithRetry(() -> {
			attempts.incrementAndGet();
			return StatusCode.EcuIsNotPresent;
		}, "test device");
		assertFalse(failed.isOK());
		assertEquals(CtreConstants.kConfigRetries, attempts.get());
	}

	private static void assertDrivingMatches(double headingRad, boolean robotOriented) {
		FakeGyroIO gyro = new FakeGyroIO();
		Swerve swerve = new Swerve(createFakeModules(), gyro);
		gyro.yawRad = headingRad;
		swerve.periodic();

		SwerveTeleOp command = new SwerveTeleOp(swerve, () -> 1.0, () -> 0.0, () -> 0.0, () -> false,
				() -> robotOriented, () -> true, value -> {
				});
		command.initialize();
		for (int i = 0; i < 30; i++) {
			command.execute();
		}
		SwerveModuleState[] actual = setpoints(swerve);

		ChassisSpeeds expectedSpeeds = robotOriented
				? new ChassisSpeeds(DriveConstants.kMaxSpeedMetersPerSecond, 0.0, 0.0)
				: ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.kMaxSpeedMetersPerSecond, 0.0, 0.0,
						new Rotation2d(headingRad));
		swerve.setModuleStates(expectedSpeeds);
		SwerveModuleState[] expected = setpoints(swerve);

		for (int i = 0; i < expected.length; i++) {
			assertEquals(expected[i].speedMetersPerSecond, actual[i].speedMetersPerSecond, 1e-6,
					"Speed mismatch at module " + i + " heading " + headingRad + " robotOriented " + robotOriented);
			double angleError = Math.abs(MathUtil
					.inputModulus(expected[i].angle.getDegrees() - actual[i].angle.getDegrees(), -180.0, 180.0));
			assertTrue(angleError < 1e-6,
					"Angle mismatch at module " + i + " heading " + headingRad + " robotOriented " + robotOriented);
		}
	}

	private static Swerve createSimSwerve() {
		SwerveModuleIO[] ios = new SwerveModuleIO[4];
		for (int i = 0; i < ios.length; i++) {
			ios[i] = new SwerveModuleIOSim();
		}
		return new Swerve(ios, new GyroIOSim(() -> 0.0));
	}

	private static SwerveTeleOp createDriveCommand(Swerve swerve, DoubleSupplier forward, DoubleSupplier turn,
			BooleanSupplier robotOriented, BooleanSupplier driverControl) {
		return new SwerveTeleOp(swerve, forward, () -> 0.0, turn, () -> false, robotOriented, driverControl, value -> {
		});
	}

	private static SwerveModuleState[] setpoints(Swerve swerve) {
		SwerveModuleState[] setpoints = new SwerveModuleState[4];
		for (int i = 0; i < setpoints.length; i++) {
			setpoints[i] = swerve.getModuleForTesting(i).getSetpointForTesting();
		}
		return setpoints;
	}

	private static FakeModuleIO[] createFakeModules() {
		FakeModuleIO[] ios = new FakeModuleIO[4];
		for (int i = 0; i < ios.length; i++) {
			ios[i] = new FakeModuleIO();
		}
		return ios;
	}

	private static class FakeModuleIO implements SwerveModuleIO {
		double drivePositionMeters;
		double driveVelocityMetersPerSec;
		double turnAbsolutePositionRad;
		double driveVoltage;
		double turnOutput;
		double turnPositionCommand;
		boolean useTurnPositionControl;
		int resetCount;
		boolean absoluteEncoderConnected = true;
		boolean driveConnected = true;
		boolean turnConnected = true;
		boolean hardwareConfigured = true;
		String hardwareFault = "";

		@Override
		public void updateInputs(SwerveModuleIOInputs inputs) {
			inputs.drivePositionMeters = drivePositionMeters;
			inputs.driveVelocityMetersPerSec = driveVelocityMetersPerSec;
			inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
			inputs.absoluteEncoderConnected = absoluteEncoderConnected;
			inputs.driveConnected = driveConnected;
			inputs.turnConnected = turnConnected;
			inputs.hardwareConfigured = hardwareConfigured;
			inputs.hardwareFault = hardwareFault;
		}

		@Override
		public void setDriveVoltage(double volts) {
			driveVoltage = volts;
		}

		@Override
		public void setTurnOutput(double output) {
			turnOutput = output;
		}

		@Override
		public boolean usesTurnPositionControl() {
			return useTurnPositionControl;
		}

		@Override
		public void setTurnPosition(double angleRad) {
			turnPositionCommand = angleRad;
		}

		@Override
		public void resetDrivePosition() {
			drivePositionMeters = 0.0;
			resetCount++;
		}
	}

	private static class FakeGyroIO implements GyroIO {
		double yawRad;
		int resetCount;
		boolean connected = true;

		@Override
		public void updateInputs(GyroIOInputs inputs) {
			inputs.yawRad = yawRad;
			inputs.connected = connected;
		}

		@Override
		public void resetYaw() {
			yawRad = 0.0;
			resetCount++;
		}
	}
}
