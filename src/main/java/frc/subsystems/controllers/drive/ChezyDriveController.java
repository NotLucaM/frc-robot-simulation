package frc.subsystems.controllers.drive;

import static frc.constants.DriveConstants.*;

import frc.robot.Commands;
import frc.robot.RobotState;
import frc.subsystems.Drive;
import frc.util.Util;

/**
 * Implements constant curvature driving. Yoinked from 254 code
 */
public class ChezyDriveController extends Drive.DriveController {

	private double mLastWheel, mQuickStopAccumulator, mNegativeInertiaAccumulator;

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		// Quick-turn if right trigger is pressed
		boolean isQuickTurn = state.driveIsQuickTurning = commands.getDriveWantsQuickTurn();
		boolean isSlowTurn = state.driveIsSlowTurning = commands.getDriveWantsSlowTurn();
		boolean isSlowMove = state.driveIsSlowMoving = commands.getDriveWantsSlowMove();
		boolean slowTurnLeft = commands.getDriveWantedSlowTurnLeft();
		boolean slowMoveBackwards = commands.getDriveWantedSlowMoveBackwards();

		double throttle = commands.getDriveWantedThrottle(), wheel = commands.getDriveWantedWheel();

		if (isSlowMove) {
			throttle = (slowMoveBackwards) ? -SLOW_MOVE_SCALAR : SLOW_MOVE_SCALAR;
		}

		double absoluteThrottle = Math.abs(throttle), absoluteWheel = Math.abs(wheel);

		wheel = Util.handleDeadBand(wheel, DEAD_BAND);
		throttle = Util.handleDeadBand(throttle, DEAD_BAND);

		double negativeWheelInertia = wheel - mLastWheel;
		mLastWheel = wheel;

		// Map linear wheel input onto a sin wave, three passes
		for (int i = 0; i < NONLINEAR_PASSES; i++) wheel = applyWheelNonLinearPass(wheel,
				WHEEL_NON_LINEARITY);

		// Negative inertia
		double negativeInertiaScalar;
		if (wheel * negativeWheelInertia > 0) {
			// If we are moving away from zero - trying to get more wheel
			negativeInertiaScalar = LOW_NEGATIVE_INERTIA_TURN_SCALAR;
		} else {
			// Going back to zero
			if (absoluteWheel > LOW_NEGATIVE_INERTIA_THRESHOLD) {
				negativeInertiaScalar = LOW_NEGATIVE_INERTIA_FAR_SCALAR;
			} else {
				negativeInertiaScalar = LOW_NEGATIVE_INERTIA_CLOSE_SCALAR;
			}
		}

		double negativeInertiaPower = negativeWheelInertia * negativeInertiaScalar;
		mNegativeInertiaAccumulator += negativeInertiaPower;

		wheel += mNegativeInertiaAccumulator;
		if (mNegativeInertiaAccumulator > 1.0) {
			mNegativeInertiaAccumulator -= 1.0;
		} else if (mNegativeInertiaAccumulator < -1.0) {
			mNegativeInertiaAccumulator += 1.0;
		} else {
			mNegativeInertiaAccumulator = 0.0;
		}

		// Quick-turn allows us to turn in place without having to be moving forward or
		// backwards
		double angularPower, overPower;
		if (isQuickTurn) {
			if (absoluteThrottle < QUICK_STOP_DEADBAND) {
				double alpha = QUICK_STOP_WEIGHT;
				mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator +
						alpha * Util.clamp01(wheel) * QUICK_STOP_SCALAR;
			}
			overPower = 1.0;
			angularPower = wheel * QUICK_TURN_SCALAR;
		} else if (isSlowTurn) {
			overPower = 1.0;
			angularPower = (slowTurnLeft) ? -SLOW_TURN_SCALAR : SLOW_TURN_SCALAR;
		} else {
			overPower = 0.0;
			angularPower = absoluteThrottle * wheel * TURN_SENSITIVITY - mQuickStopAccumulator;
			if (mQuickStopAccumulator > 1.0) {
				mQuickStopAccumulator -= 1.0;
			} else if (mQuickStopAccumulator < -1.0) {
				mQuickStopAccumulator += 1.0;
			} else {
				mQuickStopAccumulator = 0.0;
			}
		}

		double linearPower = throttle;

		double rightPower, leftPower;
		rightPower = leftPower = linearPower;
		leftPower += angularPower;
		rightPower -= angularPower;

		if (leftPower > 1.0) {
			rightPower -= overPower * (leftPower - 1.0);
			leftPower = 1.0;
		} else if (rightPower > 1.0) {
			leftPower -= overPower * (rightPower - 1.0);
			rightPower = 1.0;
		} else if (leftPower < -1.0) {
			rightPower += overPower * (-1.0 - leftPower);
			leftPower = -1.0;
		} else if (rightPower < -1.0) {
			leftPower += overPower * (-1.0 - rightPower);
			rightPower = -1.0;
		}

		outputs.leftOutput.setPercentOutput(leftPower * 0.7);
		outputs.rightOutput.setPercentOutput(rightPower * 0.7);
	}

	private double applyWheelNonLinearPass(double wheel, double wheelNonLinearity) {
		return Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
	}
}
