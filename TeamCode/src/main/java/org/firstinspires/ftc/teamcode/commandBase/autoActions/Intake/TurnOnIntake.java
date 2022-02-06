package org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

/**
 * Turn on the intake while making the scoring mechanism go to the intake state.
 */
public class TurnOnIntake implements action {
	protected Robot robot;
	protected boolean isComplete = false;
	protected boolean forward;

	public TurnOnIntake(Robot robot, boolean forward) {
		this.robot = robot;
		this.forward = forward;
	}

	@Override
	public void startAction() {
		robot.bucketSys.setState(Deposit.depositStates.COLLECTION);
		robot.Deposit.setState(Deposit.depositStates.COLLECTION);
		if (forward) {
			robot.Intake.setState(Intake.intakeStates.ON);
		} else {
			robot.Intake.setState(Intake.intakeStates.REVERSE);
		}
		isComplete = true;
	}

	@Override
	public void runAction() {

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
