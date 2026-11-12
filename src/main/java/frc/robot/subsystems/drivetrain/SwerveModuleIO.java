package frc.robot.subsystems.drivetrain;

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

		/* Raw REV encoder voltage; the CTRE path leaves these at zero. */
		public double absoluteEncoderVoltage = 0.0;
		public double absoluteEncoderReferenceVoltage = 0.0;

		/*
		 * Each implementation must set the flags it can prove. Defaults are false so a
		 * new IO that forgets a status check is treated as unhealthy, not healthy.
		 */
		public boolean driveConnected = false;
		public boolean turnConnected = false;
		public boolean hardwareConfigured = false;
		public boolean absoluteEncoderConnected = false;
		/* False means only non-essential telemetry is stale; do not disable motors. */
		public boolean telemetryValid = true;
		/*
		 * Empty when the hardware is healthy; otherwise a short human-readable reason.
		 */
		public String hardwareFault = "";
	}

	/** Reads all of the above from the hardware. Called every 20 ms. */
	void updateInputs(SwerveModuleIOInputs inputs);

	/** Drive motor voltage command, in volts. */
	void setDriveVoltage(double volts);

	/** Steering motor command as a fraction of full scale. */
	void setTurnOutput(double output);

	/**
	 * True if this hardware runs its own steering position loop. The CTRE path
	 * does; the REV and simulated paths use the shared PID in SwerveModule. This is
	 * a genuinely optional capability, so it has a safe default.
	 */
	default boolean usesTurnPositionControl() {
		return false;
	}

	/**
	 * Target steering angle in radians; only called when the above is true. An IO
	 * that claims vendor steering must override this instead of silently doing
	 * nothing.
	 */
	default void setTurnPosition(double angleRad) {
		throw new UnsupportedOperationException("This swerve module IO does not support vendor steering");
	}

	/** Re-zeroes the drive encoder after a wheel change. */
	void resetDrivePosition();
}
