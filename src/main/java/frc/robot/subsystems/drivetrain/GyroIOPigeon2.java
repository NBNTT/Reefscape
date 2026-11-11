package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CtreConstants;

/**
 * Pigeon 2 implementation. The same gyro is used on the REV and CTRE robots.
 * Disconnect, timeout, and fault states come from the Phoenix status signals.
 */
public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 gyro;
	private final StatusSignal<Angle> yaw;
	private final StatusSignal<AngularVelocity> yawRate;

	public GyroIOPigeon2(int id, String canBus) {
		gyro = canBus.isEmpty() ? new Pigeon2(id) : new Pigeon2(id, canBus);
		yaw = gyro.getYaw();
		yawRate = gyro.getAngularVelocityZWorld();

		StatusCode frequencyStatus = BaseStatusSignal.setUpdateFrequencyForAll(CtreConstants.kStatusUpdateHz, yaw,
				yawRate);
		StatusCode busStatus = gyro.optimizeBusUtilization();
		if (!frequencyStatus.isOK() || !busStatus.isOK()) {
			DriverStation.reportWarning("Pigeon 2 bus setup warning; gyro data may be slower than expected", false);
		}
		// Start with the robot's current direction as heading zero.
		resetYaw();
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		StatusCode refreshStatus = BaseStatusSignal.refreshAll(yaw, yawRate);
		inputs.yawRad = yaw.getValueAsDouble();
		inputs.connected = refreshStatus.isOK() && BaseStatusSignal.isAllGood(yaw, yawRate)
				&& Double.isFinite(inputs.yawRad);
	}

	@Override
	public void resetYaw() {
		StatusCode status = gyro.getConfigurator().setYaw(0.0);
		if (!status.isOK()) {
			DriverStation.reportError("Failed to reset Pigeon 2 yaw: " + status.getName(), false);
		}
	}
}
