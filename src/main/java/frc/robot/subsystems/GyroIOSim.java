package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

/**
 * Simulated gyro. It adds up the chassis rotation measured from the swerve
 * modules so field-oriented driving and odometry behave like the real robot in
 * simulation.
 */
public class GyroIOSim implements GyroIO {
	/** The robot loop runs at 50 Hz, so each update is 20 ms. */
	private static final double kDt = 0.02;

	private final DoubleSupplier angularVelocitySupplier;
	private double yawRad = 0.0;

	/** The supplier returns the robot's current rotation rate in radians/sec. */
	public GyroIOSim(DoubleSupplier angularVelocitySupplier) {
		this.angularVelocitySupplier = angularVelocitySupplier;
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		yawRad = MathUtil.inputModulus(yawRad + angularVelocitySupplier.getAsDouble() * kDt, -Math.PI, Math.PI);
		inputs.yawRad = yawRad;
		inputs.connected = true;
	}
}
