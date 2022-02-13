package org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.CommandDeposit2;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.GOING_IN;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.IN;

/**
 * Puts the slides back in the robot.
 */
public class GoToInState implements action {

	public static final double DEPOSIT_DURATION = CommandDeposit2.DEPOSIT_DURATION;
	protected boolean isComplete = false;
	protected Deposit.depositStates state = Deposit.depositStates.DEPOSITING;
	protected double TIME_FOR_COMPLETION = 300;
	Robot robot;
	ElapsedTime timer = new ElapsedTime();

	public GoToInState(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {

		robot.bucketSys.setState(state);
		robot.Deposit.setState(state);
		robot.Intake.setState(Intake.intakeStates.OFF);

	}

	@Override
	public void runAction() {
		switch (state) {
			case DISARMED:
				break;
			case IN:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					isComplete = true;
				}
				break;
			case COLLECTION:
				break;
			case GOING_TO_HIGH:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_HIGH;
				}
				timer.reset();
				break;
			case GOING_TO_MID:
				break;
			case GOING_TO_LOW:
				break;
			case AT_HIGH:
				if (timer.milliseconds() > TIME_FOR_COMPLETION) {
				}
				break;
			case AT_MID:
				break;
			case AT_LOW:
				break;
			case DEPOSITING:
				timer.reset();
				state = GOING_IN;
				break;
			case GOING_IN:
				if (timer.milliseconds() > CommandDeposit2.GOING_IN_DURATION) {
					state = IN;
					timer.reset();
				}
				break;
		}

		robot.bucketSys.setState(state);
		robot.Deposit.setState(state);
	}

	@Override
	public void stopAction() {

	}

	@Override
	public boolean isActionComplete() {
		return isComplete;
	}

	@Override
	public boolean isActionPersistent() {
		return false;
	}

	@Override
	public boolean isAMultipleAction() {
		return false;
	}
}
