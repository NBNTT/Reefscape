package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.Test;

/**
 * Smoke tests for the drivetrain. These use simulated modules, so they run on a
 * laptop and do not need a robot.
 */
class SwerveTest {
	@Test
	void drivesForwardInSimulation() {
		Swerve swerve = createSimSwerve();
		// Commands set their drivetrain state every cycle, so do the same here.
		for (int i = 0; i < 60; i++) {
			swerve.setModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));
			swerve.periodic();
		}

		// In 1.2 s at 1 m/s the robot should have traveled about 1.2 m.
		assertTrue(swerve.getPose().getX() > 0.5,
				"Expected the simulated robot to move forward, pose was " + swerve.getPose());
	}

	@Test
	void rotatesInSimulation() {
		Swerve swerve = createSimSwerve();
		for (int i = 0; i < 60; i++) {
			swerve.setModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
			swerve.periodic();
		}
		assertTrue(Math.abs(swerve.getHeading().getRadians()) > 0.1,
				"Expected the simulated robot to rotate, heading was " + swerve.getHeading() + " speeds "
						+ swerve.getRobotRelativeSpeeds());
	}

	@Test
	void locksWheelsInX() {
		Swerve swerve = createSimSwerve();

		// Give the steering PID time to settle (100 cycles = 2 s).
		for (int i = 0; i < 100; i++) {
			swerve.lockWheels();
			swerve.periodic();
		}

		// FL and BR point one way, FR and BL point the other; together they form an X.
		double[] expectedDegrees = {135.0, 45.0, -135.0, 135.0};
		SwerveModuleState[] states = swerve.getModuleStates();
		for (int i = 0; i < states.length; i++) {
			double error = Math
					.abs(MathUtil.inputModulus(states[i].angle.getDegrees() - expectedDegrees[i], -180.0, 180.0));
			assertTrue(error < 10.0, "Module " + i + " should be at " + expectedDegrees[i] + " degrees but was "
					+ states[i].angle.getDegrees());
		}
	}

	/**
	 * Four simulated modules plus a gyro driven by the simulated chassis rotation.
	 */
	private static Swerve createSimSwerve() {
		SwerveModuleIO[] moduleIOs = new SwerveModuleIO[4];
		for (int i = 0; i < moduleIOs.length; i++) {
			moduleIOs[i] = new SwerveModuleIOSim();
		}
		Swerve[] holder = new Swerve[1];
		Swerve swerve = new Swerve(moduleIOs, new GyroIOSim(
				() -> holder[0] == null ? 0.0 : holder[0].getRobotRelativeSpeeds().omegaRadiansPerSecond));
		holder[0] = swerve;
		return swerve;
	}
}
