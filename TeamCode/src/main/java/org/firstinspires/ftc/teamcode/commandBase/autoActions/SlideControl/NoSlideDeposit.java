package org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

public class NoSlideDeposit implements action {

	private static final double DEPOSIT_DURATION = 370;
	protected boolean isComplete = false;
	protected Deposit.depositStates state = Deposit.depositStates.AT_LOWEST;
	protected double TIME_FOR_COMPLETION = 300;
	Robot robot;
	ElapsedTime timer = new ElapsedTime();

	public NoSlideDeposit(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {

		robot.bucketSys.setState(state);
		robot.Deposit.setState(state);
		robot.Intake.setState(Intake.intakeStates.OFF);

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
