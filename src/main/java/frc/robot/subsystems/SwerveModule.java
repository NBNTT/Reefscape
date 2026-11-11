package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import org.littletonrobotics.junction.Logger;

/**
 * One swerve module's shared logic: optimize the setpoint, run the steering
 * loop, and send normalized outputs to the hardware layer.
 *
 * <p>
 * The module angle always comes from the absolute encoder, never from the motor
 * encoder, so a gear slip cannot make the steering wrong. The specific motors
 * and encoder are hidden behind {@link SwerveModuleIO}.
 */
public class SwerveModule {
	/** 0-3, used only to make the log keys unique. */
	private final int index;
	/** The motor/encoder implementation for this module. */
	private final SwerveModuleIO io;
	/** Latest hardware readings, also sent to AdvantageKit. */
	private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
	/** Steering controller; the motor controller just receives its output. */
	private final PIDController turningPID;
	/** Where this module is trying to point. */
	private SwerveModuleState setpoint = new SwerveModuleState();

	public SwerveModule(int index, SwerveModuleIO io) {
		this.index = index;
		this.io = io;

		turningPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
		// Tell the PID that -pi and +pi are the same angle.
		turningPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	/** Reads hardware without logging. Used once during construction. */
	public void refreshInputs() {
		io.updateInputs(inputs);
	}

	/**
	 * Called from {@link Swerve#periodic()}. Reads the hardware, runs the steering
	 * loop, and logs the result.
	 */
	public void periodic() {
		refreshInputs();
		applyTurnControl();
		Logger.processInputs("Drive/Module" + index, inputs);
		Logger.recordOutput("Drive/Module" + index + "/Setpoint", setpoint);

		// How far the wheel still is from the target, and how far off the
		// requested drive speed is. Useful for tuning and for spotting a
		// binding or slipping module.
		double steeringError = MathUtil.inputModulus(setpoint.angle.getRadians() - inputs.turnAbsolutePositionRad,
				-Math.PI, Math.PI);
		Logger.recordOutput("Drive/Module" + index + "/SteeringErrorRad", steeringError);
		Logger.recordOutput("Drive/Module" + index + "/DriveSpeedErrorMetersPerSec",
				setpoint.speedMetersPerSecond - inputs.driveVelocityMetersPerSec);
	}

	/**
	 * Holds the wheel where it is now. Used at startup and by {@link #stop()}, so
	 * stopping does not make the wheel snap back to zero.
	 */
	public void holdCurrentAngle() {
		setpoint = new SwerveModuleState(0.0, new Rotation2d(inputs.turnAbsolutePositionRad));
	}

	/** Measured speed and steering angle. */
	public SwerveModuleState getState() {
		return new SwerveModuleState(inputs.driveVelocityMetersPerSec, new Rotation2d(inputs.turnAbsolutePositionRad));
	}

	/** Measured distance traveled and steering angle; used by odometry. */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(inputs.drivePositionMeters, new Rotation2d(inputs.turnAbsolutePositionRad));
	}

	/**
	 * Stores a new request. The wheel is allowed to turn either way, so
	 * {@code optimize} picks the shorter path. Steering is applied immediately and
	 * again in {@link #periodic()}.
	 */
	public void setDesiredState(SwerveModuleState state) {
		// Copy before optimize so a shared/static state array is never mutated.
		SwerveModuleState optimized = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
		Rotation2d currentAngle = new Rotation2d(inputs.turnAbsolutePositionRad);
		optimized.optimize(currentAngle);
		// While the wheel is still turning, reduce drive power so it does not
		// push sideways and scrub the carpet.
		optimized.cosineScale(currentAngle);
		setpoint = optimized;
		io.setDriveOutput(
				MathUtil.clamp(optimized.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond, -1.0, 1.0));
		// Apply steering immediately instead of waiting for the next periodic().
		applyTurnControl();
	}

	/**
	 * Sends the steering command. The CTRE path runs its own position loop; the REV
	 * and simulated paths use the shared PID.
	 */
	private void applyTurnControl() {
		if (io.usesTurnPositionControl()) {
			io.setTurnPosition(setpoint.angle.getRadians());
			return;
		}

		// The current gains are P-only, so it is safe to run this again after a
		// command updates the setpoint in the same cycle.
		double turnOutput = turningPID.calculate(inputs.turnAbsolutePositionRad, setpoint.angle.getRadians());
		io.setTurnOutput(MathUtil.clamp(turnOutput, -1.0, 1.0));
	}

	/** Stops drive and holds the current steering angle. */
	public void stop() {
		io.setDriveOutput(0.0);
		io.setTurnOutput(0.0);
		holdCurrentAngle();
	}

	/** Re-zeroes the drive encoder after a wheel change. */
	public void resetDrivePosition() {
		io.resetDrivePosition();
	}
}
