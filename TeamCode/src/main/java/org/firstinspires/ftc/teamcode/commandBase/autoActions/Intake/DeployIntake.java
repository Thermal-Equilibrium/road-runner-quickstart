package org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.OFF;

public class DeployIntake implements action {


	protected Robot robot;
	protected boolean isComplete;
	protected ElapsedTime timer = new ElapsedTime();
	protected double time = 0.25;

	public DeployIntake(Robot robot) {
		this.robot = robot;
	}


	@Override
	public void startAction() {
		timer.reset();
		robot.Intake.setState(Intake.intakeStates.REVERSE);
	}

	@Override
	public void runAction() {
		if (timer.seconds() > time) isComplete = true;
	}

	@Override
	public void stopAction() {
		robot.Intake.setState(OFF);

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
