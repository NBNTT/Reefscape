package frc.robot.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Pure tests for deadband rescaling and the squared response curve. */
class SwerveTeleOpTest {
	@Test
	void deadbandIsZero() {
		assertEquals(0.0, SwerveTeleOp.shapeInput(0.05, 0.1), 1e-12);
		assertEquals(0.0, SwerveTeleOp.shapeInput(-0.05, 0.1), 1e-12);
		assertEquals(0.0, SwerveTeleOp.shapeInput(0.1, 0.1), 1e-12);
	}

	@Test
	void justOutsideDeadbandIsContinuous() {
		double shaped = SwerveTeleOp.shapeInput(0.1001, 0.1);
		assertTrue(shaped > 0.0);
		assertTrue(shaped < 1e-6);
	}

	@Test
	void fullStickReachesOne() {
		assertEquals(1.0, SwerveTeleOp.shapeInput(1.0, 0.1), 1e-12);
		assertEquals(-1.0, SwerveTeleOp.shapeInput(-1.0, 0.1), 1e-12);
	}

	@Test
	void rawInputIsClamped() {
		assertEquals(1.0, SwerveTeleOp.shapeInput(1.2, 0.1), 1e-12);
		assertEquals(-1.0, SwerveTeleOp.shapeInput(-1.2, 0.1), 1e-12);
	}

	@Test
	void nonFiniteInputIsTreatedAsZero() {
		assertEquals(0.0, SwerveTeleOp.shapeInput(Double.NaN, 0.1), 1e-12);
		assertEquals(0.0, SwerveTeleOp.shapeInput(Double.POSITIVE_INFINITY, 0.1), 1e-12);
		assertEquals(0.0, SwerveTeleOp.shapeInput(Double.NEGATIVE_INFINITY, 0.1), 1e-12);
	}

	@Test
	void lowStickIsFinerButStillMoves() {
		double shaped = SwerveTeleOp.shapeInput(0.5, 0.1);
		assertTrue(shaped > 0.0);
		assertTrue(shaped < 0.5);
	}

	@Test
	void signIsPreserved() {
		assertEquals(-SwerveTeleOp.shapeInput(0.5, 0.1), SwerveTeleOp.shapeInput(-0.5, 0.1), 1e-12);
	}
}
