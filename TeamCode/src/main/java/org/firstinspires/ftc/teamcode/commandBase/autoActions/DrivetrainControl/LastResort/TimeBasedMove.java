package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.LastResort;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.Coefficients.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients;
import org.firstinspires.ftc.teamcode.Controls.SISOControls.RobustPID;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

public class TimeBasedMove implements action {

	Robot robot;
	Vector3D robotPowers;
	ElapsedTime timer = new ElapsedTime();
	double cutoffTime = 0;
	protected Vector3D initalPose = new Vector3D();
	PIDFCoefficients coefficients;

	RobustPID pid;


	public TimeBasedMove(Robot robot, Vector3D robotPowers, double cutoffTime) {
		this.robot = robot;
		this.robotPowers = robotPowers;
		this.cutoffTime = cutoffTime;
		if (isCompBot) {
			coefficients = controllerCoefficients.compBotTurn;
		} else {
			coefficients = controllerCoefficients.protoBotTurn;
		}

	}

	@Override
	public void startAction() {
		timer.reset();
		this.initalPose = robot.getRobotPose();
		pid =  new RobustPID(coefficients, initalPose.getAngleRadians(), 3, 0.02, Math.toRadians(1));
	}

	@Override
	public void runAction() {

//
//		if (robotPowers.getAngleRadians() == 0) {
//			double output = pid.calculateLinearAngle(robot.getRobotPose().getAngleRadians());
//			robotPowers.setAngleRad(output);
//		}

		if (timer.seconds() > cutoffTime) {
			robot.driveTrain.STOP();
		} else {
			robot.driveTrain.robotRelative(robotPowers);
		}



	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return timer.seconds() > cutoffTime;
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
