package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.subsystems.Swerve;

/**
 * Wires the robot together: the drivetrain, the driver controller, and the
 * button actions.
 */
public class RobotContainer {
	private final Swerve swerve = new Swerve();
	private final CommandXboxController driver = new CommandXboxController(OIConstants.kDriverControllerPort);
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();
	/** Shared with SwerveTeleOp; left bumper flips it. */
	private boolean robotOriented = false;

	public RobotContainer() {
		// Drive with the left/right sticks; this command is always the fallback.
		swerve.setDefaultCommand(new SwerveTeleOp(swerve, driver, () -> robotOriented));

		// Drivetrain-only auto choices. Do Nothing is the safe default.
		autoChooser.setDefaultOption("Do Nothing", Autos.doNothing());
		autoChooser.addOption("Drive Forward", Autos.driveForward(swerve, 1.0, 2.0));
		SmartDashboard.putData("Auto", autoChooser);

		// Driving, X-lock, and the toggle are allowed in Teleop and Test only.
		Trigger driverControl = RobotModeTriggers.teleop().or(RobotModeTriggers.test());
		// Hold A to lock the wheels in an X.
		driver.a().and(driverControl).whileTrue(Commands.run(swerve::lockWheels, swerve));
		// Tap the left bumper to switch between field-oriented and robot-oriented.
		driver.leftBumper().and(driverControl).onTrue(Commands.runOnce(() -> robotOriented = !robotOriented));
	}

	/** Returns the autonomous command selected on the dashboard. */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
