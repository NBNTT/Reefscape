package frc.robot.subsystems;

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
		// Ask for 1 m/s forward, then run 60 robot cycles (about 1.2 s).
		swerve.setModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));

		for (int i = 0; i < 60; i++) {
			swerve.periodic();
		}

		// In 1.2 s at 1 m/s the robot should have traveled about 1.2 m.
		assertTrue(swerve.getPose().getX() > 0.5,
				"Expected the simulated robot to move forward, pose was " + swerve.getPose());
	}

	@Test
	void locksWheelsInX() {
		Swerve swerve = createSimSwerve();
		swerve.lockWheels();

		// Give the steering PID time to settle (100 cycles = 2 s).
		for (int i = 0; i < 100; i++) {
			swerve.periodic();
		}

		// FL and BR point one way, FR and BL point the other; together they form an X.
		double[] expectedDegrees = {45.0, -45.0, -45.0, 45.0};
		SwerveModuleState[] states = swerve.getModuleStates();
		for (int i = 0; i < states.length; i++) {
			double error = Math
					.abs(MathUtil.inputModulus(states[i].angle.getDegrees() - expectedDegrees[i], -180.0, 180.0));
			assertTrue(error < 10.0, "Module " + i + " should be at " + expectedDegrees[i] + " degrees but was "
					+ states[i].angle.getDegrees());
		}
	}

	/** Four simulated modules plus a gyro that never rotates. */
	private static Swerve createSimSwerve() {
		SwerveModuleIO[] moduleIOs = new SwerveModuleIO[4];
		for (int i = 0; i < moduleIOs.length; i++) {
			moduleIOs[i] = new SwerveModuleIOSim();
		}
		return new Swerve(moduleIOs, new GyroIOSim(() -> 0.0));
	}
}
