package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The one place the physical controller is mapped to named driver actions.
 *
 * <p>
 * Controller: 8BitDo Ultimate 2 Wireless in Xbox mode.
 * <ul>
 * <li>Left stick: translate.</li>
 * <li>Right stick X: rotate.</li>
 * <li>Right bumper: slow mode and rumble.</li>
 * <li>Left bumper: toggle field-oriented / robot-oriented.</li>
 * <li>A: X-lock while held.</li>
 * <li>Start (+/Menu) or Back (-/View): reset heading.</li>
 * </ul>
 *
 * Swapping controllers or buttons belongs here, not in the drivetrain or its
 * commands.
 */
public final class DriverControls {
	private final CommandXboxController controller;

	public DriverControls(int port) {
		controller = new CommandXboxController(port);
	}

	/** Forward stick, positive forward. */
	public double translateX() {
		return -controller.getLeftY();
	}

	/** Left stick, positive left. */
	public double translateY() {
		return -controller.getLeftX();
	}

	/** Right stick, positive counter-clockwise. */
	public double rotate() {
		return -controller.getRightX();
	}

	/** Right bumper held. */
	public boolean slowMode() {
		return controller.rightBumper().getAsBoolean();
	}

	/** Edge-triggered heading reset; Start is primary, Back is the old binding. */
	public Trigger resetHeading() {
		return controller.start().or(controller.back());
	}

	/** A held for X-lock. */
	public Trigger xLock() {
		return controller.a();
	}

	/** Left bumper toggles field-oriented / robot-oriented. */
	public Trigger toggleRobotOriented() {
		return controller.leftBumper();
	}

	/** Rumble feedback used by slow mode. */
	public void setRumble(double value) {
		controller.getHID().setRumble(value > 0.0 ? RumbleType.kRightRumble : RumbleType.kBothRumble, value);
	}
}
