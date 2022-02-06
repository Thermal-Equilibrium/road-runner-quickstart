package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.gamepadEnhancements.ButtonPress;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_LOW;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_MID;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.COLLECTION;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.GOING_IN;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.GOING_TO_HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.GOING_TO_LOW;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.GOING_TO_MID;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.IN;

public class CommandDeposit implements teleopAction {

	protected final double DEPOSIT_DURATION = CommandDeposit2.DEPOSIT_DURATION;
	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;
	protected Deposit.depositStates state = IN;
	protected Deposit.depositStates stateOfTarget = IN;
	protected boolean initialRunHasOccurred = false;
	protected boolean isRunning = false;
	protected boolean actionIsComplete = false;
	protected ElapsedTime timer = new ElapsedTime();

	protected ButtonPress depositButton = new ButtonPress();
	protected ButtonPress intakeButton = new ButtonPress();


	protected Vector3D poseAtRelease = new Vector3D();

	protected final double DISTANCE_FOR_SLIDES_DOWN = 15;

	public CommandDeposit(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {
		timer.reset();
		initialRunHasOccurred = true;
	}

	@Override
	public void periodic() {

		depositButton.button(slidesDownButton());

		switch (state) {
			case DISARMED:
			case COLLECTION:
				break;
			case IN:
				// cringe imo
				if (timer.milliseconds() > DEPOSIT_DURATION) {
					actionIsComplete = true;
					reset();
				}
				break;
			case GOING_TO_HIGH:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_HIGH;
				}
				timer.reset();
				break;
			case GOING_TO_MID:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_MID;
				}
				timer.reset();
				break;
			case GOING_TO_LOW:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_LOW;
				}
				timer.reset();
				break;
			case AT_HIGH:
			case AT_MID:
			case AT_LOW:
				transitionToDeposit();
				poseAtRelease = robot.getRobotPose();
				break;
			case DEPOSITING:
				if (robot.getRobotPose().distanceToPose(poseAtRelease) > DISTANCE_FOR_SLIDES_DOWN || gamepad1.right_bumper) {
					state = GOING_IN;
					timer.reset();
				}
				break;
			case GOING_IN:
				if (timer.milliseconds() > DEPOSIT_DURATION * 2) {
					state = IN;
					timer.reset();
				}
				break;
		}

		System.out.println("current state is " + state);

		robot.Deposit.setState(state);
		robot.bucketSys.setState(state);
	}

	protected void transitionToDeposit() {
		if (activeDepositButton()) {//if (robot.Deposit.isSlideWithinTolerance() && timer.milliseconds() > DEPOSIT_DURATION) {
			state = Deposit.depositStates.DEPOSITING;
			timer.reset();
		}
	}

	@Override
	public boolean isComplete() {
		return false;
	}

	@Override
	public boolean shouldRun() {
		boolean intakeButtonState = gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5;
		intakeButton.button(intakeButtonState);

		if (isRunning) {
			return true;
		} else if (intakeButtonState) {
			state = COLLECTION;
		} else if (intakeButton.release()) {
			state = IN;
		}


		boolean high = highButton();
		boolean mid = midButton();
		boolean low = lowButton();

		if (high || mid || low) {
			isRunning = true;

			if (high) {
				state = GOING_TO_HIGH;
				System.out.println("high!!");

			} else if (mid) {
				state = GOING_TO_MID;
				System.out.println("mid!!");

			} else {
				state = GOING_TO_LOW;
				System.out.println("low!!");

			}
		}

		robot.Deposit.setState(state);
		robot.bucketSys.setState(state);

		return isRunning;
	}

	@Override
	public void reset() {

		isRunning = false;
		initialRunHasOccurred = false;
		actionIsComplete = false;
		stateOfTarget = IN;
		timer.reset();
	}

	@Override
	public boolean hasPerformedInitialRun() {
		return initialRunHasOccurred;
	}


	public boolean highButton() {
		return gamepad1.triangle;
	}

	public boolean midButton() {
		return gamepad1.square;
	}

	public boolean lowButton() {
		return gamepad1.cross;
	}

	public boolean activeDepositButton() {
		return gamepad1.left_bumper;
	}

	public boolean slidesDownButton() {
		return gamepad1.right_bumper;
	}


}
