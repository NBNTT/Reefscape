package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * Pigeon 2 implementation. The same gyro is used on the REV and CTRE robots.
 */
public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 gyro;

	public GyroIOPigeon2(int id, String canBus) {
		gyro = canBus.isEmpty() ? new Pigeon2(id) : new Pigeon2(id, canBus);
		// Start with the robot's current direction as heading zero.
		gyro.reset();
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.yawRad = gyro.getRotation2d().getRadians();
		inputs.connected = gyro.getYaw().getStatus() == StatusCode.OK;
	}

	@Override
	public void resetYaw() {
		gyro.reset();
	}
}
