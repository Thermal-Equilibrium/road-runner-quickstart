package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FindStaticFrictionTurn implements action {
	Robot robot;
	double power = 0;
	double iteration = 0.0005;
	boolean isComplete = false;
	Vector3D initialPosition;

	public FindStaticFrictionTurn(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		initialPosition = robot.getRobotPose();
	}

	@Override
	public void runAction() {

		robot.driveTrain.robotRelative(0,0,power);
		power += iteration;
		if (robot.getRobotPose().angle.getRadians() > Math.abs(0.1)) {
			isComplete = true;
		}
		Dashboard.packet.put("turn power", power);


	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		if (isComplete) {
			System.out.println("minimum motor power turn is " + power);
		}
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
