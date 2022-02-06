package org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;

/**
 * Deposits the freight once the slides are out.
 */
public class DepositFreight implements action {

	protected Robot robot;
	protected boolean isComplete = false;
	protected ElapsedTime timer = new ElapsedTime();
	protected double depositTime = 130;

	public DepositFreight(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		robot.bucketSys.setState(Deposit.depositStates.DEPOSITING);
		robot.Deposit.setState(Deposit.depositStates.DEPOSITING);
		timer.reset();
	}

	@Override
	public void runAction() {
		if (timer.milliseconds() > depositTime) {
			isComplete = true;
		}
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
