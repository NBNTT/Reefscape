package frc.robot.subsystems;

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

	public SwerveModuleIOSparkMax(SwerveModuleConfig config) {
		this.config = config;

		// Build the entire config first, then send it once. Do not call
		// configure() halfway through: anything set afterward never reaches the
		// motor.
		driveMotor = new SparkMax(config.driveId(), MotorType.kBrushless);
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig.voltageCompensation(12.0);
		driveConfig.smartCurrentLimit(40);
		driveConfig.inverted(config.driveInverted());
		driveConfig.idleMode(IdleMode.kBrake);
		// Convert motor rotations to meters and RPM to meters per second.
		driveConfig.encoder.positionConversionFactor(config.driveRotToMeters());
		driveConfig.encoder.velocityConversionFactor(config.driveRotToMeters() / 60.0);
		reportIfError(driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
				"Drive SparkMax " + config.driveId());

		// The steering angle comes from the absolute encoder, so this motor does
		// not need an encoder conversion.
		turnMotor = new SparkMax(config.turnId(), MotorType.kBrushless);
		SparkMaxConfig turnConfig = new SparkMaxConfig();
		turnConfig.voltageCompensation(12.0);
		turnConfig.smartCurrentLimit(40);
		turnConfig.inverted(config.turnInverted());
		turnConfig.idleMode(IdleMode.kBrake);
		reportIfError(turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
				"Turn SparkMax " + config.turnId());

		absoluteEncoder = new AnalogInput(config.encoderId());
		// Distance is measured from where the robot starts.
		driveMotor.getEncoder().setPosition(0.0);
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
		boolean encoderConnected = referenceVoltage > 0.1 && encoderVoltage > 0.05
				&& encoderVoltage < referenceVoltage - 0.05;
		inputs.absoluteEncoderConnected = encoderConnected;
		inputs.turnAbsolutePositionRad = absoluteEncoderAngle(encoderVoltage, referenceVoltage);
		inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
		inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
		inputs.turnTempCelsius = turnMotor.getMotorTemperature();
	}

	/**
	 * Converts the raw 0-5 V encoder voltage into a wheel angle. The offset makes
	 * "wheel pointing straight forward" read as zero; the modulus keeps the angle
	 * between -pi and +pi.
	 */
	private double absoluteEncoderAngle(double voltage, double referenceVoltage) {
		double angle = referenceVoltage > 0.1 ? voltage / referenceVoltage * 2.0 * Math.PI : 0.0;
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
		driveMotor.getEncoder().setPosition(0.0);
	}

	/** A failed SparkMax configuration is silent unless we report it. */
	private static void reportIfError(REVLibError error, String device) {
		if (error != REVLibError.kOk) {
			DriverStation.reportError("Failed to configure " + device + ": " + error, false);
		}
	}
}
