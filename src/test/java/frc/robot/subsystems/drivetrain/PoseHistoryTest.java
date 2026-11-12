package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

/** Ring-buffer and interpolation checks for the delayed-vision pose history. */
class PoseHistoryTest {
	@Test
	void interpolatesAndClampsToTheOldestAndNewestSamples() {
		PoseHistory history = new PoseHistory(3);
		history.reset(1.0, new Pose2d(0.0, 0.0, new Rotation2d()));
		history.add(2.0, new Pose2d(2.0, 0.0, new Rotation2d()));

		assertEquals(1.0, history.get(1.5).getX(), 1e-9);
		assertEquals(0.0, history.get(0.5).getX(), 1e-9);
		assertEquals(2.0, history.get(2.5).getX(), 1e-9);
	}

	@Test
	void evictsTheOldestSampleWhenFull() {
		PoseHistory history = new PoseHistory(2);
		history.reset(1.0, new Pose2d(1.0, 0.0, new Rotation2d()));
		history.add(2.0, new Pose2d(2.0, 0.0, new Rotation2d()));
		history.add(3.0, new Pose2d(3.0, 0.0, new Rotation2d()));

		assertEquals(2.0, history.get(1.0).getX(), 1e-9);
		assertEquals(3.0, history.get(99.0).getX(), 1e-9);
	}

	@Test
	void resetDropsOldHistory() {
		PoseHistory history = new PoseHistory(2);
		history.reset(1.0, new Pose2d(1.0, 0.0, new Rotation2d()));
		history.add(2.0, new Pose2d(2.0, 0.0, new Rotation2d()));
		history.reset(10.0, new Pose2d(10.0, 0.0, new Rotation2d()));

		assertEquals(10.0, history.get(1.0).getX(), 1e-9);
		assertEquals(10.0, history.get(11.0).getX(), 1e-9);
	}

	@Test
	void emptyHistoryReturnsNull() {
		assertNull(new PoseHistory(2).get(1.0));
	}
}
