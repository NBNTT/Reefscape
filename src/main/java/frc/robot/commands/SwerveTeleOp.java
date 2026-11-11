package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;

/**
 * Default drivetrain command. It runs whenever no other command is using the
 * drivetrain.
 *
 * <p>
 * Controls: right bumper = slow mode with rumble, left bumper = field/robot
 * oriented toggle (bound in {@link frc.robot.RobotContainer}), A = X-lock
 * (bound in {@link frc.robot.RobotContainer}).
 */
public class SwerveTeleOp extends Command {
	private final Swerve swerve;
	private final CommandXboxController driver;
	/** True when the driver wants robot-oriented instead of field-oriented. */
	private final BooleanSupplier robotOriented;
	/** Limits how fast the requested speed can change. */
	private final SlewRateLimiter forwardLimiter;
	private final SlewRateLimiter strafeLimiter;
	private final SlewRateLimiter turnLimiter;

	public SwerveTeleOp(Swerve swerve, CommandXboxController driver, BooleanSupplier robotOriented) {
		this.swerve = swerve;
		this.driver = driver;
		this.robotOriented = robotOriented;

		forwardLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);
		strafeLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationMetersPerSecondSquared);
		turnLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared);

		// No other command may drive the drivetrain while this one is active.
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		// Start from zero whenever the command starts or resumes after X-lock.
		forwardLimiter.reset(0.0);
		strafeLimiter.reset(0.0);
		turnLimiter.reset(0.0);
	}

	@Override
	public void execute() {
		// Default commands run in every mode; only the driver may drive in teleop or
		// test.
		if (!DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled()) {
			setRumble(false);
			swerve.stopModules();
			forwardLimiter.reset(0.0);
			strafeLimiter.reset(0.0);
			turnLimiter.reset(0.0);
			return;
		}

		// WPILib axes: +X is forward and +Y is left. The sticks report the
		// opposite sign, so negate them here.
		double forward = -driver.getLeftY();
		double strafe = -driver.getLeftX();
		double turn = -driver.getRightX();

		// Use the combined stick distance so pure sideways strafing is not
		// mistaken for "no input".
		if (Math.hypot(forward, strafe) < OIConstants.kDeadband) {
			forward = 0.0;
			strafe = 0.0;
		}
		if (Math.abs(turn) < OIConstants.kDeadband) {
			turn = 0.0;
		}

		// Old stick boost, kept so the driving feel matches the old robot.
		forward *= forward > 0 ? 1.0 / 0.8 : 1.0 / 0.9;
		strafe *= 1.0 / 0.9;
		forward = MathUtil.clamp(forward, -1.0, 1.0);
		strafe = MathUtil.clamp(strafe, -1.0, 1.0);
		turn = MathUtil.clamp(turn, -1.0, 1.0);

		// Right bumper: scale everything down and give the driver feedback.
		boolean slowMode = driver.rightBumper().getAsBoolean();
		setRumble(slowMode);
		if (slowMode) {
			forward *= OIConstants.kSlowModeFactor;
			strafe *= OIConstants.kSlowModeFactor;
			turn *= OIConstants.kSlowModeFactor;
		}

		// Convert stick fractions to real units, then limit how fast they change.
		forward = forwardLimiter.calculate(forward * DriveConstants.kMaxSpeedMetersPerSecond);
		strafe = strafeLimiter.calculate(strafe * DriveConstants.kMaxSpeedMetersPerSecond);
		turn = turnLimiter.calculate(turn * DriveConstants.kMaxAngularSpeedRadiansPerSecond);

		// Field-oriented uses the robot's current heading; robot-oriented does not.
		ChassisSpeeds speeds = robotOriented.getAsBoolean()
				? new ChassisSpeeds(forward, strafe, turn)
				: ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, swerve.getHeading());
		swerve.setModuleStates(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		// Release the wheels and stop the rumble when another command takes over.
		swerve.stopModules();
		setRumble(false);
	}

	/** Right-side rumble while slow mode is held; both sides off otherwise. */
	private void setRumble(boolean slowMode) {
		driver.getHID().setRumble(slowMode ? RumbleType.kRightRumble : RumbleType.kBothRumble,
				slowMode ? OIConstants.kRumble : 0.0);
	}
}
