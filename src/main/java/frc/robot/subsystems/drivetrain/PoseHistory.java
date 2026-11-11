package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Bounded pose history for delayed vision. The pose estimator itself only knows
 * the current pose, so a camera frame is compared with the robot's estimated
 * pose at the frame timestamp.
 */
final class PoseHistory {
	private final double[] timestamps;
	private final Pose2d[] poses;
	private int size = 0;
	private int next = 0;

	PoseHistory(int capacity) {
		if (capacity <= 0) {
			throw new IllegalArgumentException("Pose history capacity must be positive");
		}
		timestamps = new double[capacity];
		poses = new Pose2d[capacity];
	}

	/** Drops old data, for example after a pose or heading reset. */
	void reset(double timestampSeconds, Pose2d pose) {
		size = 0;
		next = 0;
		add(timestampSeconds, pose);
	}

	/** Adds a sample; timestamps are forced to stay strictly increasing. */
	void add(double timestampSeconds, Pose2d pose) {
		if (pose == null) {
			return;
		}
		double timestamp = timestampSeconds;
		if (size > 0) {
			double last = timestampAt(size - 1);
			if (timestamp <= last) {
				timestamp = last + 1e-6;
			}
		}
		timestamps[next] = timestamp;
		poses[next] = pose;
		next = (next + 1) % timestamps.length;
		if (size < timestamps.length) {
			size++;
		}
	}

	/** Interpolated pose at the timestamp, or null when no history exists. */
	Pose2d get(double timestampSeconds) {
		if (size == 0) {
			return null;
		}
		if (timestampSeconds <= timestampAt(0)) {
			return poses[index(0)];
		}
		if (timestampSeconds >= timestampAt(size - 1)) {
			return poses[index(size - 1)];
		}
		for (int i = 0; i < size - 1; i++) {
			double startTime = timestampAt(i);
			double endTime = timestampAt(i + 1);
			if (timestampSeconds <= endTime) {
				double alpha = (timestampSeconds - startTime) / (endTime - startTime);
				return poses[index(i)].interpolate(poses[index(i + 1)], alpha);
			}
		}
		return poses[index(size - 1)];
	}

	private double timestampAt(int logicalIndex) {
		return timestamps[index(logicalIndex)];
	}

	private int index(int logicalIndex) {
		return (next - size + logicalIndex + timestamps.length) % timestamps.length;
	}
}
