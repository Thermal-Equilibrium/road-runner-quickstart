package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FollowTrajectory implements action {

	boolean isComplete = false;

	Robot robot;
	Trajectory trajectory;

	public FollowTrajectory(Robot robot, Trajectory trajectory) {
		this.robot = robot;
		this.trajectory = trajectory;
	}

	@Override
	public void startAction() {
		this.robot.driveTrain.mecanumDrive.followTrajectoryAsync(this.trajectory);
	}

	@Override
	public void runAction() {
		if (this.robot.driveTrain.mecanumDrive.isBusy()) return;
		isComplete = true;
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
		return true;
	}

	@Override
	public boolean isAMultipleAction() {
		return false;
	}
}
