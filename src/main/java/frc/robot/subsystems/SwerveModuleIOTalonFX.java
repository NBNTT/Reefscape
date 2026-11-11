package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveModuleConfig;

/**
 * Future CTRE MK5n implementation: Kraken X60 drive, Kraken X44 turn, CANcoder
 * absolute encoder.
 *
 * <p>
 * Not active on the 2024 robot. Fill the CTRE IDs, offsets, wheel diameter,
 * gear ratio, and current limits in {@link Constants} before switching
 * {@code Constants.kHardware} to {@code CTRE_MK5N}.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
	private final TalonFX driveMotor;
	private final TalonFX turnMotor;
	private final CANcoder absoluteEncoder;
	private final SwerveModuleConfig config;

	/*
	 * Cached CTRE signals; reading them every cycle is cheaper than asking twice.
	 */
	private final StatusSignal<Angle> drivePosition;
	private final StatusSignal<AngularVelocity> driveVelocity;
	private final StatusSignal<Voltage> driveVoltage;
	private final StatusSignal<Current> driveCurrent;
	private final StatusSignal<Temperature> driveTemp;

	private final StatusSignal<Voltage> turnVoltage;
	private final StatusSignal<Current> turnCurrent;
	private final StatusSignal<Temperature> turnTemp;
	private final StatusSignal<Angle> turnAbsolutePosition;

	public SwerveModuleIOTalonFX(SwerveModuleConfig config) {
		this.config = config;

		driveMotor = createTalonFX(config.driveId());
		turnMotor = createTalonFX(config.turnId());
		absoluteEncoder = createCANcoder(config.encoderId());

		// Drive motor: brake mode, direction, current limits, and gearing.
		// SensorToMechanismRatio makes getPosition() report wheel rotations
		// instead of motor rotations.
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveConfig.MotorOutput.Inverted = config.driveInverted()
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.Feedback.SensorToMechanismRatio = config.driveGearRatio();
		driveMotor.getConfigurator().apply(driveConfig);

		// Steering motor: brake, direction, and current limits. The angle comes
		// from the CANcoder, so this motor's own encoder is not used.
		TalonFXConfiguration turnConfig = new TalonFXConfiguration();
		turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		turnConfig.MotorOutput.Inverted = config.turnInverted()
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		turnConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turnConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		// Let the TalonFX steer using the CANcoder directly. This runs the
		// steering loop inside the motor controller instead of at 50 Hz on the
		// RoboRIO. Gains are placeholders until they are tuned on the new robot.
		turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		turnConfig.Feedback.FeedbackRemoteSensorID = config.encoderId();
		turnConfig.Feedback.SensorToMechanismRatio = 1.0;
		turnConfig.Slot0.kP = ModuleConstants.kCtreTurnKp;
		turnConfig.Slot0.kI = ModuleConstants.kCtreTurnKi;
		turnConfig.Slot0.kD = ModuleConstants.kCtreTurnKd;
		turnMotor.getConfigurator().apply(turnConfig);

		// Put the calibration offset and direction in the CANcoder config so the
		// TalonFX sees the same calibrated angle as the rest of the code.
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.SensorDirection = config.encoderInverted()
				? SensorDirectionValue.Clockwise_Positive
				: SensorDirectionValue.CounterClockwise_Positive;
		// A reversed sensor flips the raw angle, so the offset must flip too.
		double offsetRotations = config.encoderOffsetRad() / (2.0 * Math.PI);
		encoderConfig.MagnetSensor.MagnetOffset = config.encoderInverted() ? -offsetRotations : offsetRotations;
		absoluteEncoder.getConfigurator().apply(encoderConfig);

		// Grab the signals once so updateInputs() can refresh them in one call.
		drivePosition = driveMotor.getPosition();
		driveVelocity = driveMotor.getVelocity();
		driveVoltage = driveMotor.getMotorVoltage();
		driveCurrent = driveMotor.getSupplyCurrent();
		driveTemp = driveMotor.getDeviceTemp();

		turnVoltage = turnMotor.getMotorVoltage();
		turnCurrent = turnMotor.getSupplyCurrent();
		turnTemp = turnMotor.getDeviceTemp();
		turnAbsolutePosition = absoluteEncoder.getAbsolutePosition();

		// Swerve data should be at least as fresh as the 50 Hz robot loop.
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, drivePosition, driveVelocity, driveVoltage, driveCurrent,
				driveTemp, turnVoltage, turnCurrent, turnTemp, turnAbsolutePosition);
		driveMotor.optimizeBusUtilization();
		turnMotor.optimizeBusUtilization();
		absoluteEncoder.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// Ask all signals for a fresh value in one batch.
		BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveVoltage, driveCurrent, driveTemp, turnVoltage,
				turnCurrent, turnTemp, turnAbsolutePosition);

		// getPosition/getVelocity are in wheel rotations and rotations per second
		// because of SensorToMechanismRatio; multiply by circumference for meters.
		double wheelCircumference = Math.PI * config.wheelDiameterMeters();
		inputs.drivePositionMeters = drivePosition.getValueAsDouble() * wheelCircumference;
		inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble() * wheelCircumference;
		inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
		inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
		inputs.driveTempCelsius = driveTemp.getValueAsDouble();

		// The CANcoder config already applied the offset and direction, so this
		// is the calibrated module angle. Convert rotations to radians and wrap.
		double angle = turnAbsolutePosition.getValueAsDouble() * 2.0 * Math.PI;
		inputs.turnAbsolutePositionRad = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
		inputs.turnVelocityRadPerSec = 0.0;
		inputs.turnAppliedVolts = turnVoltage.getValueAsDouble();
		inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
		inputs.turnTempCelsius = turnTemp.getValueAsDouble();
		inputs.absoluteEncoderConnected = turnAbsolutePosition.getStatus() == StatusCode.OK;
	}

	@Override
	public boolean usesTurnPositionControl() {
		return true;
	}

	@Override
	public void setTurnPosition(double angleRad) {
		// PositionVoltage wants rotations. Pick the target rotation closest to the
		// current one so the wheel does not take the long way around.
		double currentRotations = turnAbsolutePosition.getValueAsDouble();
		double targetRotations = angleRad / (2.0 * Math.PI);
		double wrappedTarget = currentRotations + MathUtil.inputModulus(targetRotations - currentRotations, -0.5, 0.5);
		turnMotor.setControl(new PositionVoltage(wrappedTarget).withSlot(0));
	}

	@Override
	public void setDriveOutput(double output) {
		driveMotor.set(MathUtil.clamp(output, -1.0, 1.0));
	}

	@Override
	public void setTurnOutput(double output) {
		turnMotor.set(MathUtil.clamp(output, -1.0, 1.0));
	}

	@Override
	public void resetDrivePosition() {
		driveMotor.setPosition(0.0);
	}

	/** Empty CAN bus name means the roboRIO bus; otherwise use the CANivore. */
	private static TalonFX createTalonFX(int id) {
		return Constants.CAN.kCanBus.isEmpty() ? new TalonFX(id) : new TalonFX(id, Constants.CAN.kCanBus);
	}

	private static CANcoder createCANcoder(int id) {
		return Constants.CAN.kCanBus.isEmpty() ? new CANcoder(id) : new CANcoder(id, Constants.CAN.kCanBus);
	}
}
