package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for one swerve module. The drivetrain logic only sees
 * meters, radians, volts, and amps, so the REV, CTRE, and simulated modules can
 * be swapped without touching {@link SwerveModule}.
 */
public interface SwerveModuleIO {
	@AutoLog
	class SwerveModuleIOInputs {
		/* Drive wheel: distance traveled, speed, and electrical readings. */
		public double drivePositionMeters = 0.0;
		public double driveVelocityMetersPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double driveCurrentAmps = 0.0;
		public double driveTempCelsius = 0.0;

		/* Steering: absolute angle is the real source of truth. */
		public double turnAbsolutePositionRad = 0.0;
		public double turnAppliedVolts = 0.0;
		public double turnCurrentAmps = 0.0;
		public double turnTempCelsius = 0.0;

		/* False usually means the absolute encoder is unplugged or miswired. */
		public boolean absoluteEncoderConnected = false;
	}

	/** Reads all of the above from the hardware. Called every 20 ms. */
	default void updateInputs(SwerveModuleIOInputs inputs) {
	}

	/** Drive motor voltage command, in volts. */
	default void setDriveVoltage(double volts) {
	}

	/** Steering motor command as a fraction of full scale. */
	default void setTurnOutput(double output) {
	}

	/**
	 * True if this hardware runs its own steering position loop. The CTRE path
	 * does; the REV and simulated paths use the shared PID in SwerveModule.
	 */
	default boolean usesTurnPositionControl() {
		return false;
	}

	/** Target steering angle in radians; only called when the above is true. */
	default void setTurnPosition(double angleRad) {
	}

	/** Re-zeroes the drive encoder after a wheel change. */
	default void resetDrivePosition() {
	}
}
