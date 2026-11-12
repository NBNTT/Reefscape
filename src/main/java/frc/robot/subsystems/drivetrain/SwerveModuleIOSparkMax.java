package frc.robot.subsystems.drivetrain;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RevConstants;
import frc.robot.Constants.SwerveModuleConfig;

/**
 * REV MK4n implementation for the old robot: two SparkMax controllers and an
 * analog absolute encoder plugged into the roboRIO.
 */
public class SwerveModuleIOSparkMax implements SwerveModuleIO {
	private final SparkMax driveMotor;
	private final SparkMax turnMotor;
	private final AnalogInput absoluteEncoder;
	private final SwerveModuleConfig config;
	private final boolean driveConfigured;
	private final boolean turnConfigured;
	private final String hardwareFault;
	private int faultPollCounter = 0;
	private boolean driveCanFault = false;
	private boolean turnCanFault = false;

	public SwerveModuleIOSparkMax(SwerveModuleConfig config) {
		this.config = config;

		// Build the entire config first, then send it once. Do not call
		// configure() halfway through: anything set afterward never reaches the
		// motor.
		driveMotor = new SparkMax(config.driveId(), MotorType.kBrushless);
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig.voltageCompensation(RevConstants.kVoltageCompensationVolts);
		driveConfig.smartCurrentLimit(RevConstants.kDriveSmartCurrentLimitAmps);
		driveConfig.inverted(config.driveInverted());
		driveConfig.idleMode(IdleMode.kBrake);
		// Convert motor rotations to meters and RPM to meters per second.
		driveConfig.encoder.positionConversionFactor(config.driveRotToMeters());
		driveConfig.encoder.velocityConversionFactor(config.driveRotToMeters() / 60.0);
		driveConfigured = configure(driveMotor, driveConfig, "Drive SparkMax " + config.driveId());

		// The steering angle comes from the absolute encoder, so this motor does
		// not need an encoder conversion.
		turnMotor = new SparkMax(config.turnId(), MotorType.kBrushless);
		SparkMaxConfig turnConfig = new SparkMaxConfig();
		turnConfig.voltageCompensation(RevConstants.kVoltageCompensationVolts);
		turnConfig.smartCurrentLimit(RevConstants.kTurnSmartCurrentLimitAmps);
		turnConfig.inverted(config.turnInverted());
		turnConfig.idleMode(IdleMode.kBrake);
		turnConfigured = configure(turnMotor, turnConfig, "Turn SparkMax " + config.turnId());

		absoluteEncoder = new AnalogInput(config.encoderId());
		// Distance is measured from where the robot starts.
		resetDrivePosition();

		hardwareFault = buildFault();
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// Drive readings are already in meters and meters per second because of
		// the conversion factors above.
		inputs.drivePositionMeters = driveMotor.getEncoder().getPosition();
		inputs.driveVelocityMetersPerSec = driveMotor.getEncoder().getVelocity();
		inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
		inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
		inputs.driveTempCelsius = driveMotor.getMotorTemperature();

		// The analog encoder's voltage tells us the wheel angle. A reading at
		// the very bottom or top of the range usually means it is unplugged.
		double encoderVoltage = absoluteEncoder.getAverageVoltage();
		double referenceVoltage = RobotController.getVoltage5V();
		boolean referenceOk = Double.isFinite(referenceVoltage)
				&& referenceVoltage >= RevConstants.kMinReferenceVoltageVolts;
		boolean encoderConnected = referenceOk && Double.isFinite(encoderVoltage)
				&& encoderVoltage > RevConstants.kEncoderMinVolts
				&& encoderVoltage < referenceVoltage - RevConstants.kEncoderMaxMarginVolts;
		inputs.absoluteEncoderConnected = encoderConnected;
		inputs.absoluteEncoderVoltage = encoderVoltage;
		inputs.absoluteEncoderReferenceVoltage = referenceVoltage;
		inputs.turnAbsolutePositionRad = absoluteEncoderAngle(encoderVoltage, referenceVoltage, encoderConnected);
		inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
		inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
		inputs.turnTempCelsius = turnMotor.getMotorTemperature();

		inputs.hardwareConfigured = driveConfigured && turnConfigured;
		inputs.hardwareFault = hardwareFault;

		// shortcut: REV CAN faults are sampled every 100 ms, not every 20 ms.
		// Upgrade this if a faster motor-disconnect response is required.
		if (faultPollCounter == 0) {
			driveCanFault = hasCanFault(driveMotor);
			turnCanFault = hasCanFault(turnMotor);
		}
		faultPollCounter = (faultPollCounter + 1) % RevConstants.kFaultPollFrames;
		inputs.driveConnected = driveConfigured && !driveCanFault;
		inputs.turnConnected = turnConfigured && !turnCanFault;
	}

	/**
	 * Converts the raw 0-5 V encoder voltage into a wheel angle. The offset makes
	 * "wheel pointing straight forward" read as zero; the modulus keeps the angle
	 * between -pi and +pi. An invalid reading returns NaN so it cannot steer.
	 */
	private double absoluteEncoderAngle(double voltage, double referenceVoltage, boolean connected) {
		if (!connected) {
			return Double.NaN;
		}
		double angle = voltage / referenceVoltage * 2.0 * Math.PI;
		angle += config.encoderOffsetRad();
		angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
		return config.encoderInverted() ? -angle : angle;
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
	}

	@Override
	public void setTurnOutput(double output) {
		turnMotor.set(MathUtil.clamp(output, -1.0, 1.0));
	}

	@Override
	public void resetDrivePosition() {
		REVLibError error = driveMotor.getEncoder().setPosition(0.0);
		if (error != REVLibError.kOk) {
			DriverStation.reportWarning("Failed to reset drive encoder on SparkMax " + config.driveId() + ": " + error,
					false);
		}
	}

	/** Applies one complete configuration, retrying transient failures. */
	private static boolean configure(SparkMax motor, SparkMaxConfig config, String device) {
		REVLibError error = REVLibError.kError;
		for (int attempt = 1; attempt <= RevConstants.kConfigRetries; attempt++) {
			error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
			if (error == REVLibError.kOk) {
				return true;
			}
			if (attempt < RevConstants.kConfigRetries) {
				Timer.delay(RevConstants.kConfigRetryDelaySeconds);
			}
		}
		DriverStation.reportError(
				"Failed to configure " + device + " after " + RevConstants.kConfigRetries + " attempts: " + error,
				false);
		return false;
	}

	private String buildFault() {
		if (driveConfigured && turnConfigured) {
			return "";
		}
		return (driveConfigured ? "" : "drive config failed; ") + (turnConfigured ? "" : "turn config failed");
	}

	/** A failed fault query is treated as a CAN fault, not as good news. */
	private static boolean hasCanFault(SparkMax motor) {
		try {
			return motor.getFaults().can;
		} catch (RuntimeException exception) {
			return true;
		}
	}
}
