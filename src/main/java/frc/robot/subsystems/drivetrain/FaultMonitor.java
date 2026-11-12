package frc.robot.subsystems.drivetrain;

/**
 * Shared debounce and recovery counter for one health signal. A single bad
 * frame is visible immediately through {@link #isValid()}, while a sustained
 * failure is reported through {@link #isHealthy()} only after the configured
 * number of frames.
 */
final class FaultMonitor {
	private final int faultFrames;
	private final int recoveryFrames;

	private boolean healthy = true;
	private boolean valid = false;
	private int invalidFrames = 0;
	private int goodFrames = 0;
	private int recoveryCount = 0;
	private String invalidReason = "";
	private String faultReason = "";

	FaultMonitor(int faultFrames, int recoveryFrames) {
		if (faultFrames <= 0 || recoveryFrames <= 0) {
			throw new IllegalArgumentException("Fault and recovery frame counts must be positive");
		}
		this.faultFrames = faultFrames;
		this.recoveryFrames = recoveryFrames;
	}

	/** One frame of input; a null reason is treated as an empty reason. */
	void update(boolean valid, String reason) {
		this.valid = valid;
		invalidReason = valid || reason == null ? "" : reason;

		if (valid) {
			invalidFrames = 0;
			if (!healthy) {
				goodFrames++;
				if (goodFrames >= recoveryFrames) {
					healthy = true;
					faultReason = "";
					goodFrames = 0;
					recoveryCount++;
				}
			}
			return;
		}

		goodFrames = 0;
		invalidFrames++;
		if (healthy && invalidFrames >= faultFrames) {
			healthy = false;
			faultReason = invalidReason;
		} else if (!healthy) {
			faultReason = invalidReason;
		}
	}

	boolean isHealthy() {
		return healthy;
	}

	/** False on the first bad frame, before the fault debounce expires. */
	boolean isValid() {
		return valid;
	}

	/** Empty while healthy, otherwise the sustained-fault reason. */
	String getFaultReason() {
		return healthy ? "" : faultReason;
	}

	/** Empty when the last frame was good. */
	String getInvalidReason() {
		return invalidReason;
	}

	int getRecoveryCount() {
		return recoveryCount;
	}
}
