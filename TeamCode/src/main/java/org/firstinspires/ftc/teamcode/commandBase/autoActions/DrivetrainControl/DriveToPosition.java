package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import org.firstinspires.ftc.teamcode.Controls.MIMOControls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DriveToPosition implements action {

	Robot robot;
	Vector3D referencePose;
	MecanumDriveController controller;
	protected double cutOffTime = 5;
	protected boolean MAX_ACCEL_LETS_GO = false;

	public DriveToPosition(Robot robot, Vector3D referencePose) {
		this.robot = robot;
		this.referencePose = referencePose;
		controller = new MecanumDriveController();
	}

	public DriveToPosition(Robot robot, Vector3D referencePose, double cutOffTime, boolean useMaxAccel) {
		this.robot = robot;
		this.referencePose = referencePose;
		controller = new MecanumDriveController();
		this.cutOffTime = cutOffTime;
		this.MAX_ACCEL_LETS_GO = useMaxAccel;
	}


	@Override
	public void startAction() {
		controller.resetTimer();
	}

	@Override
	public void runAction() {
		Vector3D fieldRelative;
		if (MAX_ACCEL_LETS_GO) {
			fieldRelative = controller.calculate(referencePose,new Vector3D(),robot.getRobotPose(),robot.getVelocity()).rotateBy(robot.getRobotPose().getAngleDegrees());
		} else {
			fieldRelative = controller.calculateSpeedRamped(referencePose, robot.getRobotPose(), robot.getVelocity());
		}
		robot.driveTrain.robotRelative(fieldRelative);
	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return controller.followingIsComplete() || controller.getTime() > cutOffTime;
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
