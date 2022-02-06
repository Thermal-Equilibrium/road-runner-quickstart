package org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.CommandDeposit2;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_LOW;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_MID;

public class GoToMidDeposit implements action {
	protected boolean isComplete = false;
	protected Deposit.depositStates state = Deposit.depositStates.GOING_TO_MID;
	protected double TIME_FOR_COMPLETION = CommandDeposit2.DEPOSIT_DURATION;
	Robot robot;
	ElapsedTime timer = new ElapsedTime();

	public GoToMidDeposit(Robot robot) {
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
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_MID;
				}
				break;
			case GOING_TO_LOW:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_LOW;
				}
				break;
			case AT_HIGH:
			case AT_MID:
			case AT_LOW:
				if (timer.milliseconds() > TIME_FOR_COMPLETION) {
					isComplete = true;
				}
				break;
			case DEPOSITING:
				break;
			case GOING_IN:
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
