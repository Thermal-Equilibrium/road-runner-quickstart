package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.Coefficients.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients;
import org.firstinspires.ftc.teamcode.Controls.SISOControls.RobustPID;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

public class Turn implements action {

	Robot robot;
	RobustPID pid;

	PIDFCoefficients coefficients;
	double targetAngle;
	boolean isComplete = false;
	ElapsedTime timer = new ElapsedTime();
	protected double cutoffTimeSeconds = 10;

	public Turn(Robot robot, double targetAngle) {
		this.robot = robot;
		this.targetAngle = targetAngle;
		if (isCompBot) {
			coefficients = controllerCoefficients.compBotTurn;
		} else {
			coefficients = controllerCoefficients.protoBotTurn;
		}
	}

	public Turn(Robot robot, double targetAngle, double cutoffTimeSeconds) {
		this.robot = robot;
		this.targetAngle = targetAngle;
		if (isCompBot) {
			coefficients = controllerCoefficients.compBotTurn;
		} else {
			coefficients = controllerCoefficients.protoBotTurn;
		}
		this.cutoffTimeSeconds = cutoffTimeSeconds;
	}

	@Override
	public void startAction() {
		pid = new RobustPID(coefficients, targetAngle, 3, 0.02, Math.toRadians(1));

		timer.reset();
	}

	@Override
	public void runAction() {
		double output = pid.calculateLinearAngle(robot.getRobotPose().getAngleRadians());
		robot.driveTrain.robotRelative(0,0, output);
		Dashboard.packet.put("power",output);
		Dashboard.packet.put("error",pid.getError());
		double errorPercent = Math.abs(pid.getError()) / Math.abs(targetAngle);
		isComplete = ((pid.isComplete()) && pid.isStable()) || (pid.isVeryStable() && errorPercent < 0.05);
	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return isComplete || timer.seconds() > cutoffTimeSeconds;
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
