package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Swerve;

/**
 * One-shot heading reset used by the Start and Back buttons. It is
 * intentionally allowed while disabled so the driver can aim the robot before a
 * match. It is not bound while an autonomous is running.
 */
public class ResetHeading extends Command {
	private final Swerve swerve;
	private final SwerveTeleOp driveCommand;

	public ResetHeading(Swerve swerve, SwerveTeleOp driveCommand) {
		this.swerve = swerve;
		this.driveCommand = driveCommand;
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		swerve.resetHeading();
		driveCommand.resetHeadingHold();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
