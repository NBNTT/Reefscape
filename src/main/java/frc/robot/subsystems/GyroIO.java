package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for the gyro. The drivetrain only needs the robot's
 * heading, in radians, so the Pigeon 2 and the simulated gyro can be swapped.
 */
public interface GyroIO {
	@AutoLog
	class GyroIOInputs {
		/** Robot heading in radians, wrapped to -pi..pi. */
		public double yawRad = 0.0;
		/** False if the gyro is not responding. */
		public boolean connected = false;
	}

	/** Reads the current heading. Called every 20 ms. */
	default void updateInputs(GyroIOInputs inputs) {
	}

	/** Makes the robot's current direction the new heading zero. */
	default void resetYaw() {
	}
}
