package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

/**
 * Drivetrain-only autonomous foundation. Add real game autos here later; the
 * chooser in {@link frc.robot.RobotContainer} already selects from this list.
 */
public final class Autos {
	private Autos() {
	}

	/** Leaves the robot in place. This is the safe default. */
	public static Command doNothing() {
		return Commands.none();
	}

	/**
	 * Resets the pose, drives straight forward at a fixed robot-relative speed, and
	 * stops when the command ends or is interrupted.
	 */
	public static Command driveForward(Swerve swerve, double speedMetersPerSecond, double seconds) {
		return Commands.runOnce(() -> swerve.resetPose(new Pose2d()))
				.andThen(Commands
						.run(() -> swerve.setModuleStates(new ChassisSpeeds(speedMetersPerSecond, 0.0, 0.0)), swerve)
						.withTimeout(seconds))
				.finallyDo(interrupted -> swerve.stopModules());
	}
}
