package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DrivetrainTurnIdentification implements action {
	protected Robot robot;
	protected double power = 0;
	protected double max_power = 0.75;
	protected double step_time_seconds = 0.25;
	protected double total_step_time = step_time_seconds * 2;
	protected boolean isComplete = false;
	ElapsedTime timer = new ElapsedTime();

	public DrivetrainTurnIdentification(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		System.out.println("drivetrain forward: power, position, time");
		timer.reset();
	}

	@Override
	public void runAction() {
		double time = timer.seconds();
		if (time > step_time_seconds) {
			power = max_power;
		}
		robot.driveTrain.robotRelative(0,0, power);
		System.out.println("drivetrain turn: " + power + ", " + robot.getRobotPose().getAngleRadians() + ", " + time);
		if (time > total_step_time) {
			isComplete = true;
		}
	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
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
