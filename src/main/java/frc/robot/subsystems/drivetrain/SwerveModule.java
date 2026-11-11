package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HealthConstants;
import frc.robot.Constants.ModuleConstants;
import org.littletonrobotics.junction.Logger;

/**
 * One swerve module's shared logic: validate the sensors, optimize the
 * setpoint, run the drive and steering loops, and send hardware commands.
 *
 * <p>
 * The module angle always comes from the absolute encoder, never from the motor
 * encoder, so a gear slip cannot make the steering wrong. Invalid readings are
 * never used for steering, and a fault is debounced so a single noisy sample
 * does not disable the robot.
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

	/*
	 * Health state. A single bad frame stops this module immediately; a sustained
	 * fault is reported after the debounce count and disables driving.
	 */
	private boolean healthy = true;
	private boolean readingValid = false;
	private int invalidFrames = 0;
	private int recoveryFrames = 0;
	private int recoveryCount = 0;
	private double lastValidAngleRad = 0.0;
	private boolean hasLastValidAngle = false;
	private double candidateAngleRad = 0.0;
	private int candidateFrames = 0;
	private String invalidReason = "";
	private String faultReason = "";

	public SwerveModule(int index, SwerveModuleIO io) {
		this.index = index;
		this.io = io;

		turningPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
		// Tell the PID that -pi and +pi are the same angle.
		turningPID.enableContinuousInput(-Math.PI, Math.PI);

		drivePID = new PIDController(DriveConstants.kDriveKp, DriveConstants.kDriveKi, DriveConstants.kDriveKd);
		driveFeedforward = new SimpleMotorFeedforward(DriveConstants.kDriveKsVolts, DriveConstants.kDriveKvVoltsPerMps);

		// Seed the angle and health state before the first command is scheduled.
		refreshInputs();
	}

	/** Reads hardware, updates health, and logs nothing. */
	public void refreshInputs() {
		io.updateInputs(inputs);
		updateHealth();
	}

	/**
	 * Called from {@link Swerve#periodic()}. Reads the hardware, updates health,
	 * and logs it. The commands call {@link #setDesiredState(SwerveModuleState)}
	 * every cycle; that is where healthy drive and steering outputs are applied.
	 */
	public void periodic() {
		refreshInputs();
		Logger.processInputs("Drive/Module" + index, inputs);
		Logger.recordOutput("Drive/Module" + index + "/Setpoint", setpoint);
		Logger.recordOutput("Drive/Module" + index + "/Healthy", healthy);
		Logger.recordOutput("Drive/Module" + index + "/ReadingValid", readingValid);
		Logger.recordOutput("Drive/Module" + index + "/InvalidReason", invalidReason);
		Logger.recordOutput("Drive/Module" + index + "/FaultReason", getFaultReason());
		Logger.recordOutput("Drive/Module" + index + "/RecoveryCount", recoveryCount);

		if (!healthy || !readingValid) {
			// Never steer or drive from a reading we do not trust.
			neutralize();
			setpoint = new SwerveModuleState(0.0, new Rotation2d(currentAngleRad()));
			return;
		}

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
		setpoint = new SwerveModuleState(0.0, new Rotation2d(currentAngleRad()));
	}

	/** Measured speed and steering angle. */
	public SwerveModuleState getState() {
		return new SwerveModuleState(finiteOrZero(inputs.driveVelocityMetersPerSec), new Rotation2d(currentAngleRad()));
	}

	/** Measured distance traveled and steering angle; used by odometry. */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(finiteOrZero(inputs.drivePositionMeters), new Rotation2d(currentAngleRad()));
	}

	/**
	 * Applies a new request. The wheel is allowed to turn either way, so
	 * {@code optimize} picks the shorter path. Call this every command cycle.
	 */
	public void setDesiredState(SwerveModuleState state) {
		if (state == null || state.angle == null || !Double.isFinite(state.speedMetersPerSecond)
				|| !Double.isFinite(state.angle.getRadians())) {
			stop();
			return;
		}
		if (!healthy || !readingValid) {
			stop();
			return;
		}

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

	/**
	 * Stops drive and holds the current steering angle when the angle is trusted.
	 */
	public void stop() {
		setpoint = new SwerveModuleState(0.0, new Rotation2d(currentAngleRad()));
		if (healthy && readingValid) {
			io.setDriveVoltage(0.0);
			applyTurnControl();
		} else {
			// Do not ask a vendor loop to hold an angle read from a bad encoder.
			neutralize();
		}
	}

	/**
	 * Re-zeroes the drive encoder after a wheel change and refreshes the inputs.
	 */
	public void resetDrivePosition() {
		io.resetDrivePosition();
		refreshInputs();
	}

	/** True after the fault debounce, not on a single noisy frame. */
	public boolean isHealthy() {
		return healthy;
	}

	/** False on any bad encoder frame; driving is blocked immediately. */
	boolean hasValidReading() {
		return readingValid;
	}

	/** Empty when healthy; otherwise the reason the module is disabled. */
	String getFaultReason() {
		return healthy ? "" : faultReason;
	}

	/** Empty when the last frame was good; otherwise why it was rejected. */
	String getInvalidReason() {
		return invalidReason;
	}

	/** Test-only snapshot; callers cannot mutate the real setpoint. */
	SwerveModuleState getSetpointForTesting() {
		return new SwerveModuleState(setpoint.speedMetersPerSecond, setpoint.angle);
	}

	private void updateHealth() {
		String bad = validateReading();
		invalidReason = bad == null ? "" : bad;

		if (bad == null) {
			invalidFrames = 0;
			candidateFrames = 0;
			readingValid = true;
			lastValidAngleRad = inputs.turnAbsolutePositionRad;
			hasLastValidAngle = true;
			if (!healthy) {
				recoveryFrames++;
				if (recoveryFrames >= HealthConstants.kEncoderRecoveryFrames) {
					healthy = true;
					faultReason = "";
					recoveryFrames = 0;
					recoveryCount++;
				}
			}
			return;
		}

		readingValid = false;
		recoveryFrames = 0;
		invalidFrames++;
		if (healthy && invalidFrames >= HealthConstants.kEncoderFaultDebounceFrames) {
			healthy = false;
			faultReason = bad;
		} else if (!healthy) {
			faultReason = bad;
		}
	}

	/**
	 * Returns null when the frame can be used. A jump must persist at the new angle
	 * before it is accepted, so a single spike cannot steer the module.
	 */
	private String validateReading() {
		if (!inputs.hardwareConfigured) {
			candidateFrames = 0;
			return inputs.hardwareFault == null || inputs.hardwareFault.isBlank()
					? "hardware not configured"
					: inputs.hardwareFault;
		}
		if (!inputs.driveConnected) {
			candidateFrames = 0;
			return "drive disconnected";
		}
		if (!inputs.turnConnected) {
			candidateFrames = 0;
			return "turn disconnected";
		}
		if (!inputs.absoluteEncoderConnected) {
			candidateFrames = 0;
			return "absolute encoder disconnected";
		}
		if (!Double.isFinite(inputs.drivePositionMeters) || !Double.isFinite(inputs.driveVelocityMetersPerSec)) {
			candidateFrames = 0;
			return "non-finite drive data";
		}

		double angle = inputs.turnAbsolutePositionRad;
		if (!Double.isFinite(angle)) {
			candidateFrames = 0;
			return "non-finite absolute angle";
		}
		if (!hasLastValidAngle) {
			return null;
		}

		double jump = Math.abs(MathUtil.inputModulus(angle - lastValidAngleRad, -Math.PI, Math.PI));
		if (jump <= HealthConstants.kMaxAngleJumpRad) {
			candidateFrames = 0;
			return null;
		}

		double candidateJump = candidateFrames == 0
				? Double.POSITIVE_INFINITY
				: Math.abs(MathUtil.inputModulus(angle - candidateAngleRad, -Math.PI, Math.PI));
		candidateFrames = candidateJump <= HealthConstants.kMaxAngleJumpRad ? candidateFrames + 1 : 1;
		candidateAngleRad = angle;
		if (candidateFrames >= HealthConstants.kAngleJumpConfirmFrames) {
			// The new angle has been stable long enough to be real, not noise.
			candidateFrames = 0;
			return null;
		}
		return "absolute angle jump";
	}

	private void neutralize() {
		io.setDriveVoltage(0.0);
		io.setTurnOutput(0.0);
		// Drop any integral/derivative history so recovery starts clean.
		drivePID.reset();
		turningPID.reset();
	}

	private double currentAngleRad() {
		if (hasLastValidAngle) {
			return lastValidAngleRad;
		}
		return Double.isFinite(inputs.turnAbsolutePositionRad) ? inputs.turnAbsolutePositionRad : 0.0;
	}

	private static double finiteOrZero(double value) {
		return Double.isFinite(value) ? value : 0.0;
	}
}
