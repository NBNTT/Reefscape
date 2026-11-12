package frc.robot.subsystems.drivetrain;

/**
 * Shared time-based debounce and recovery counter for one health signal. A bad
 * sample is visible immediately through {@link #isValid()}, while a sustained
 * failure is reported through {@link #isHealthy()} only after the fault window.
 */
final class FaultMonitor {
	private final double faultSeconds;
	private final double recoverySeconds;

	private boolean healthy = true;
	private boolean valid = false;
	private double invalidSeconds = 0.0;
	private double goodSeconds = 0.0;
	private int recoveryCount = 0;
	private String invalidReason = "";
	private String faultReason = "";

	FaultMonitor(double faultSeconds, double recoverySeconds) {
		if (!(faultSeconds > 0.0) || !(recoverySeconds > 0.0)) {
			throw new IllegalArgumentException("Fault and recovery times must be positive");
		}
		this.faultSeconds = faultSeconds;
		this.recoverySeconds = recoverySeconds;
	}

	/** One sample; a null reason is treated as an empty reason. */
	void update(boolean valid, String reason, double dtSeconds) {
		this.valid = valid;
		invalidReason = valid || reason == null ? "" : reason;
		double dt = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.0;

		if (valid) {
			invalidSeconds = 0.0;
			if (!healthy) {
				goodSeconds += dt;
				if (goodSeconds >= recoverySeconds - 1e-9) {
					healthy = true;
					faultReason = "";
					goodSeconds = 0.0;
					recoveryCount++;
				}
			}
			return;
		}

		goodSeconds = 0.0;
		invalidSeconds += dt;
		if (healthy && invalidSeconds >= faultSeconds - 1e-9) {
			healthy = false;
			faultReason = invalidReason;
		} else if (!healthy) {
			faultReason = invalidReason;
		}
	}

	boolean isHealthy() {
		return healthy;
	}

	/** False on the first bad sample, before the fault window expires. */
	boolean isValid() {
		return valid;
	}

	/** Empty while healthy, otherwise the sustained-fault reason. */
	String getFaultReason() {
		return healthy ? "" : faultReason;
	}

	/** Empty when the last sample was good. */
	String getInvalidReason() {
		return invalidReason;
	}

	int getRecoveryCount() {
		return recoveryCount;
	}
}
