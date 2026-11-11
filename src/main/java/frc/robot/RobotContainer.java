package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.Autos;
import frc.robot.commands.drivetrain.ResetHeading;
import frc.robot.commands.drivetrain.SwerveTeleOp;
import frc.robot.subsystems.drivetrain.Swerve;

/**
 * Wires the robot together: the drivetrain, the semantic driver controls, and
 * the button actions.
 */
public class RobotContainer {
	private final Swerve swerve = new Swerve();
	private final DriverControls controls = new DriverControls(OIConstants.kDriverControllerPort);
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();
	private final SwerveTeleOp driveCommand;
	/** Driving, X-lock, and the mode toggle are allowed in Teleop and Test only. */
	private final Trigger driverControl = RobotModeTriggers.teleop().or(RobotModeTriggers.test());
	/** Shared with SwerveTeleOp; left bumper flips it. */
	private boolean robotOriented = false;

	public RobotContainer() {
		// Drive with the left/right sticks; this command is always the fallback.
		driveCommand = new SwerveTeleOp(swerve, controls::translateX, controls::translateY, controls::rotate,
				controls::slowMode, () -> robotOriented, driverControl::getAsBoolean, controls::setRumble);
		swerve.setDefaultCommand(driveCommand);

		// Drivetrain-only auto choices. Do Nothing is the safe default.
		autoChooser.setDefaultOption("Do Nothing", Autos.doNothing());
		autoChooser.addOption("Drive Forward", Autos.driveForward(swerve, 1.0, 2.0));
		SmartDashboard.putData("Auto", autoChooser);

		// Hold A to lock the wheels in an X.
		controls.xLock().and(driverControl).whileTrue(Commands.run(swerve::lockWheels, swerve));
		// Start resets the current direction to field-forward. Back is also bound
		// because the original 2025 code used Back for the gyro reset. The command
		// is explicitly allowed while disabled; it is not bound in Autonomous.
		Command resetHeading = new ResetHeading(swerve, driveCommand);
		controls.resetHeading().and(RobotModeTriggers.disabled().or(driverControl)).onTrue(resetHeading);
		// Tap the left bumper to switch between field-oriented and robot-oriented.
		controls.toggleRobotOriented().and(driverControl)
				.onTrue(Commands.runOnce(() -> robotOriented = !robotOriented));
	}

	/** Returns the autonomous command selected on the dashboard. */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
