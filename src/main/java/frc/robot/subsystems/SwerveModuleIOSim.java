package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;

/**
 * Simple simulated module. The drive motor follows the commanded voltage
 * instantly and the steering motor is first-order. Good enough to exercise the
 * real drivetrain, odometry, and command code in desktop simulation and tests.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
	/** The robot loop runs at 50 Hz, so each update is 20 ms. */
	private static final double kDt = 0.02;
	/** Arbitrary steering speed; only used to make the simulated wheel move. */
	private static final double kTurnSpeedRadPerSec = 20.0;

	private double drivePositionMeters = 0.0;
	private double driveVelocityMetersPerSec = 0.0;
	private double turnPositionRad = 0.0;
	private double turnVelocityRadPerSec = 0.0;
	private double driveVoltage = 0.0;
	private double turnOutput = 0.0;

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// Pretend 12 V means top speed; the real robot's feedforward/PID makes
		// this a reasonable approximation for tests.
		driveVelocityMetersPerSec = driveVoltage / 12.0 * DriveConstants.kMaxSpeedMetersPerSecond;
		drivePositionMeters += driveVelocityMetersPerSec * kDt;

		// Steering is a simple rate-limited angle change.
		turnVelocityRadPerSec = turnOutput * kTurnSpeedRadPerSec;
		turnPositionRad = MathUtil.inputModulus(turnPositionRad + turnVelocityRadPerSec * kDt, -Math.PI, Math.PI);

		inputs.drivePositionMeters = drivePositionMeters;
		inputs.driveVelocityMetersPerSec = driveVelocityMetersPerSec;
		inputs.driveAppliedVolts = driveVoltage;
		inputs.driveCurrentAmps = Math.abs(driveVoltage) / 12.0 * 40.0;
		inputs.driveTempCelsius = 25.0;

		inputs.turnAbsolutePositionRad = turnPositionRad;
		inputs.turnAppliedVolts = turnOutput * 12.0;
		inputs.turnCurrentAmps = Math.abs(turnOutput) * 20.0;
		inputs.turnTempCelsius = 25.0;
		inputs.absoluteEncoderConnected = true;
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveVoltage = MathUtil.clamp(volts, -12.0, 12.0);
	}

	@Override
	public void setTurnOutput(double output) {
		turnOutput = MathUtil.clamp(output, -1.0, 1.0);
	}

	@Override
	public void resetDrivePosition() {
		drivePositionMeters = 0.0;
	}
}
