// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The robot's main loop.
 *
 * <p>
 * Extending {@link LoggedRobot} (instead of TimedRobot) is what lets
 * AdvantageKit record and replay data. The constructor builds the robot,
 * {@link #robotInit()} starts logging, and {@link #robotPeriodic()} runs the
 * command scheduler.
 */
public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	private final RobotContainer robotContainer;

	public Robot() {
		// Build hardware and controls before logging starts.
		robotContainer = new RobotContainer();
	}

	@Override
	public void robotInit() {
		if (RobotBase.isReal()) {
			// Real robot: write a .wpilog to a USB stick and also stream to
			// AdvantageScope over network tables.
			Logger.addDataReceiver(new WPILOGWriter("/U/logs/"));
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			// Simulation: stream to AdvantageScope only.
			Logger.addDataReceiver(new NT4Publisher());
		}
		Logger.start();
	}

	@Override
	public void robotPeriodic() {
		// Runs subsystem periodic() methods and the active commands.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		// Do not let an autonomous command keep running into teleop.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
}
