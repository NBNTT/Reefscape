package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drivetrain.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Default drivetrain command. It runs whenever no other command is using the
 * drivetrain.
 *
 * <p>
 * Inputs are semantic suppliers supplied by {@link frc.robot.DriverControls};
 * this command does not know which physical buttons produced them. Slow mode
 * scales the request and rumbles, and a faulted drivetrain stays stopped.
 */
public class SwerveTeleOp extends Command {
	private final Swerve swerve;
	private final DoubleSupplier forwardSupplier;
	private final DoubleSupplier strafeSupplier;
	private final DoubleSupplier turnSupplier;
	private final BooleanSupplier slowModeSupplier;
	private final BooleanSupplier robotOriented;
	private final BooleanSupplier driverControlEnabled;
	private final DoubleConsumer rumble;

	/** Fixed-step limits on how fast the requested speed can change. */
	private double forwardLimited = 0.0;
	private double strafeLimited = 0.0;
	private double turnLimited = 0.0;

	/**
	 * Separate from the steering PID; holds a heading when the turn stick is
	 * released.
	 */
	private final PIDController headingPID;
	private boolean headingHoldActive = false;
	private double headingTargetRad = 0.0;
	private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

	/** Testable constructor: inject stick values, slow mode, and rumble. */
	public SwerveTeleOp(Swerve swerve, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier,
			DoubleSupplier turnSupplier, BooleanSupplier slowModeSupplier, BooleanSupplier robotOriented,
			BooleanSupplier driverControlEnabled, DoubleConsumer rumble) {
		this.swerve = swerve;
		this.forwardSupplier = forwardSupplier;
		this.strafeSupplier = strafeSupplier;
		this.turnSupplier = turnSupplier;
		this.slowModeSupplier = slowModeSupplier;
		this.robotOriented = robotOriented;
		this.driverControlEnabled = driverControlEnabled;
		this.rumble = rumble;

		headingPID = new PIDController(HeadingConstants.kKp, HeadingConstants.kKi, HeadingConstants.kKd);
		headingPID.enableContinuousInput(-Math.PI, Math.PI);

		// No other command may drive the drivetrain while this one is active.
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		// Start from zero whenever the command starts or resumes after X-lock.
		resetLimiters();
		resetHeadingHold();
		lastChassisSpeeds = new ChassisSpeeds();
	}

	@Override
	public void execute() {
		// Default commands run in every mode; only the driver may drive in teleop or
		// test.
		if (!driverControlEnabled.getAsBoolean() || !swerve.isDrivetrainHealthy()) {
			resetHeadingHold();
			lastChassisSpeeds = new ChassisSpeeds();
			setRumble(false);
			swerve.stopModules();
			resetLimiters();
			return;
		}

		// Deadband rescales the usable range continuously; squaring gives finer
		// low-speed control while still reaching 1.0 at full stick.
		double forward = shapeInput(forwardSupplier.getAsDouble(), OIConstants.kDeadband);
		double strafe = shapeInput(strafeSupplier.getAsDouble(), OIConstants.kDeadband);
		double turnInput = MathUtil.clamp(turnSupplier.getAsDouble(), -1.0, 1.0);
		double turnRaw = MathUtil.applyDeadband(turnInput, OIConstants.kDeadband);
		double turn = Math.copySign(turnRaw * turnRaw, turnRaw);

		// Right bumper: scale everything down and give the driver feedback.
		boolean slowMode = slowModeSupplier.getAsBoolean();
		setRumble(slowMode);
		if (slowMode) {
			forward *= OIConstants.kSlowModeFactor;
			strafe *= OIConstants.kSlowModeFactor;
			turn *= OIConstants.kSlowModeFactor;
		}

		boolean robotOrientedNow = robotOriented.getAsBoolean() || !swerve.isFieldOrientedAvailable();
		boolean manualRotation = Math.abs(turnRaw) >= HeadingConstants.kManualTurnThreshold;
		double requestedTurnRadPerSec;
		if (robotOrientedNow || manualRotation) {
			// Manual rotation always wins; heading hold is disabled in robot-oriented
			// mode and while the gyro cannot provide field heading.
			resetHeadingHold();
			requestedTurnRadPerSec = turn * DriveConstants.kMaxAngularSpeedRadiansPerSecond;
		} else {
			if (!headingHoldActive) {
				headingHoldActive = true;
				headingTargetRad = swerve.getHeading().getRadians();
				headingPID.reset();
			}
			requestedTurnRadPerSec = MathUtil.clamp(
					headingPID.calculate(swerve.getHeading().getRadians(), headingTargetRad),
					-DriveConstants.kMaxAngularSpeedRadiansPerSecond, DriveConstants.kMaxAngularSpeedRadiansPerSecond);
		}

		// Convert stick fractions to real units, then limit how fast they change.
		forward = forwardLimited = slew(forwardLimited, forward * DriveConstants.kMaxSpeedMetersPerSecond,
				DriveConstants.kMaxAccelerationMetersPerSecondSquared, DriveConstants.kLoopPeriodSeconds);
		strafe = strafeLimited = slew(strafeLimited, strafe * DriveConstants.kMaxSpeedMetersPerSecond,
				DriveConstants.kMaxAccelerationMetersPerSecondSquared, DriveConstants.kLoopPeriodSeconds);
		double turnRadPerSec = turnLimited = slew(turnLimited, requestedTurnRadPerSec,
				DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared, DriveConstants.kLoopPeriodSeconds);

		// Field-oriented uses the robot's current heading; robot-oriented does not.
		ChassisSpeeds speeds = robotOrientedNow
				? new ChassisSpeeds(forward, strafe, turnRadPerSec)
				: ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turnRadPerSec, swerve.getHeading());
		lastChassisSpeeds = speeds;
		swerve.setModuleStates(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		// Release the wheels and stop the rumble when another command takes over.
		resetHeadingHold();
		lastChassisSpeeds = new ChassisSpeeds();
		swerve.stopModules();
		setRumble(false);
	}

	/** Clears the held heading; the next neutral turn cycle captures a new one. */
	public void resetHeadingHold() {
		headingHoldActive = false;
		headingPID.reset();
	}

	/** Fixed-step acceleration limit; independent of test wall-clock time. */
	private static double slew(double current, double target, double maxRate, double dt) {
		return current + MathUtil.clamp(target - current, -maxRate * dt, maxRate * dt);
	}

	private void resetLimiters() {
		forwardLimited = 0.0;
		strafeLimited = 0.0;
		turnLimited = 0.0;
	}

	/** Test-only view of the most recent chassis command. */
	public ChassisSpeeds getLastChassisSpeedsForTesting() {
		return lastChassisSpeeds;
	}

	/** Test-only view of whether heading hold is currently active. */
	public boolean isHeadingHoldActiveForTesting() {
		return headingHoldActive;
	}

	/**
	 * Deadband-rescale then square, preserving sign and reaching 1.0 at full stick.
	 */
	static double shapeInput(double raw, double deadband) {
		double clamped = MathUtil.clamp(raw, -1.0, 1.0);
		double deadbanded = MathUtil.applyDeadband(clamped, deadband);
		return Math.copySign(deadbanded * deadbanded, deadbanded);
	}

	/** Right-side rumble while slow mode is held; both sides off otherwise. */
	private void setRumble(boolean slowMode) {
		rumble.accept(slowMode ? OIConstants.kRumble : 0.0);
	}
}
