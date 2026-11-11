package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import frc.robot.Constants.Hardware;
import frc.robot.Constants.SwerveModuleConfig;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Startup guards: the active robot must be complete, and broken wiring must
 * fail.
 */
class ConstantsTest {
	@Test
	void activeRevHardwarePassesValidation() {
		assertDoesNotThrow(Constants::validateActiveHardware);
	}

	@Test
	void unfinishedCtreConfigurationIsRejected() {
		assertThrows(IllegalStateException.class, () -> Constants.validateHardware(Hardware.CTRE_MK5N,
				Constants.DriveConstants.kCtreModuleConfigs, false));
		assertDoesNotThrow(() -> Constants.validateHardware(Hardware.CTRE_MK5N,
				Constants.DriveConstants.kCtreModuleConfigs, true));
	}

	@Test
	void duplicateCanIdsAndBadAnalogChannelsAreRejected() {
		SwerveModuleConfig first = Constants.DriveConstants.kRevModuleConfigs.get(0);

		List<SwerveModuleConfig> duplicateId = new ArrayList<>(Constants.DriveConstants.kRevModuleConfigs);
		duplicateId.set(0, copy(first, Constants.CAN.kRevFrontRightDriveId, first.encoderId(), first.encoderOffsetRad(),
				first.wheelDiameterMeters()));
		assertThrows(IllegalStateException.class,
				() -> Constants.validateHardware(Hardware.REV_MK4N, duplicateId, false));

		List<SwerveModuleConfig> badChannel = new ArrayList<>(Constants.DriveConstants.kRevModuleConfigs);
		badChannel.set(0, copy(first, first.driveId(), 7, first.encoderOffsetRad(), first.wheelDiameterMeters()));
		assertThrows(IllegalStateException.class,
				() -> Constants.validateHardware(Hardware.REV_MK4N, badChannel, false));
	}

	@Test
	void nonFiniteOffsetAndImplausibleWheelAreRejected() {
		SwerveModuleConfig first = Constants.DriveConstants.kRevModuleConfigs.get(0);

		List<SwerveModuleConfig> badOffset = new ArrayList<>(Constants.DriveConstants.kRevModuleConfigs);
		badOffset.set(0, copy(first, first.driveId(), first.encoderId(), Double.NaN, first.wheelDiameterMeters()));
		assertThrows(IllegalStateException.class,
				() -> Constants.validateHardware(Hardware.REV_MK4N, badOffset, false));

		List<SwerveModuleConfig> badWheel = new ArrayList<>(Constants.DriveConstants.kRevModuleConfigs);
		badWheel.set(0, copy(first, first.driveId(), first.encoderId(), first.encoderOffsetRad(), 0.0));
		assertThrows(IllegalStateException.class, () -> Constants.validateHardware(Hardware.REV_MK4N, badWheel, false));
	}

	private static SwerveModuleConfig copy(SwerveModuleConfig base, int driveId, int encoderId, double offset,
			double wheelDiameter) {
		return new SwerveModuleConfig(base.name(), driveId, base.turnId(), encoderId, base.driveInverted(),
				base.turnInverted(), base.encoderInverted(), offset, wheelDiameter, base.driveGearRatio());
	}
}
