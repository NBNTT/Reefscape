package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import org.littletonrobotics.junction.Logger;

/**
 * One swerve module's shared logic: optimize the setpoint, run the drive and
 * steering loops, and send hardware commands.
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
	/** Drive speed controller; uses measured wheel speed, not fixed power. */
	private final PIDController drivePID;
	/** Feedforward seeded from the calibrated top speed. */
	private final SimpleMotorFeedforward driveFeedforward;
	/** Where this module is trying to point and how fast. */
	private SwerveModuleState setpoint = new SwerveModuleState();

	public SwerveModule(int index, SwerveModuleIO io) {
		this.index = index;
		this.io = io;

		turningPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
		// Tell the PID that -pi and +pi are the same angle.
		turningPID.enableContinuousInput(-Math.PI, Math.PI);

		drivePID = new PIDController(DriveConstants.kDriveKp, DriveConstants.kDriveKi, DriveConstants.kDriveKd);
		driveFeedforward = new SimpleMotorFeedforward(DriveConstants.kDriveKsVolts, DriveConstants.kDriveKvVoltsPerMps);
	}

	/** Reads hardware without logging; also used after a drive-position reset. */
	public void refreshInputs() {
		io.updateInputs(inputs);
	}

	/**
	 * Called from {@link Swerve#periodic()}. Reads the hardware and logs it. The
	 * commands call {@link #setDesiredState(SwerveModuleState)} every cycle; that
	 * is where the drive and steering outputs are applied.
	 */
	public void periodic() {
		refreshInputs();
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
	 * Applies a new request. The wheel is allowed to turn either way, so
	 * {@code optimize} picks the shorter path. Call this every command cycle.
	 */
	public void setDesiredState(SwerveModuleState state) {
		// Copy before optimize so a shared/static state array is never mutated.
		SwerveModuleState optimized = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
		Rotation2d currentAngle = new Rotation2d(inputs.turnAbsolutePositionRad);
		optimized.optimize(currentAngle);
		// Keep the logged setpoint in -pi..pi even if a caller passed 359 degrees.
		optimized = new SwerveModuleState(optimized.speedMetersPerSecond,
				new Rotation2d(MathUtil.inputModulus(optimized.angle.getRadians(), -Math.PI, Math.PI)));
		// While the wheel is still turning, reduce drive power so it does not
		// push sideways and scrub the carpet.
		optimized.cosineScale(currentAngle);
		setpoint = optimized;
		applyDriveControl();
		applyTurnControl();
	}

	/**
	 * Drives at the requested wheel speed using measured velocity plus feedforward.
	 */
	private void applyDriveControl() {
		// Do not apply kS at a zero setpoint; otherwise a calibrated friction term
		// would make a stopped or X-locked robot creep.
		double feedforward = Math.abs(setpoint.speedMetersPerSecond) < 0.01
				? 0.0
				: driveFeedforward.calculate(setpoint.speedMetersPerSecond);
		double feedback = drivePID.calculate(inputs.driveVelocityMetersPerSec, setpoint.speedMetersPerSecond);
		io.setDriveVoltage(MathUtil.clamp(feedforward + feedback, -12.0, 12.0));
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

		double turnOutput = turningPID.calculate(inputs.turnAbsolutePositionRad, setpoint.angle.getRadians());
		io.setTurnOutput(MathUtil.clamp(turnOutput, -1.0, 1.0));
	}

	/** Stops drive and holds the current steering angle. */
	public void stop() {
		io.setDriveVoltage(0.0);
		holdCurrentAngle();
		applyTurnControl();
	}

	/**
	 * Re-zeroes the drive encoder after a wheel change and refreshes the inputs.
	 */
	public void resetDrivePosition() {
		io.resetDrivePosition();
		refreshInputs();
	}

	/** Test-only snapshot; callers cannot mutate the real setpoint. */
	SwerveModuleState getSetpointForTesting() {
		return new SwerveModuleState(setpoint.speedMetersPerSecond, setpoint.angle);
	}
}
