package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveTeleOp;
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
		assertEquals(2, swerve.getRejectedVisionCountForTesting());
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

		// The feedforward should command positive volts, not a raw percent.
		assertTrue(io.driveVoltage > 0.0);
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

		@Override
		public void updateInputs(SwerveModuleIOInputs inputs) {
			inputs.drivePositionMeters = drivePositionMeters;
			inputs.driveVelocityMetersPerSec = driveVelocityMetersPerSec;
			inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
			inputs.absoluteEncoderConnected = true;
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

		@Override
		public void updateInputs(GyroIOInputs inputs) {
			inputs.yawRad = yawRad;
			inputs.connected = true;
		}
	}
}
