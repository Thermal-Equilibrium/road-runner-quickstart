package org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ThreeWheelOdometry;

@Config
public class TuneOdometryTrackWidth implements action {
	double headingAccumulator = 0;
	double lastHeading = 0;
	public static double NUM_TURNS = 5;
	Robot robot;
	Gamepad gamepad1;
	public TuneOdometryTrackWidth(Robot robot,Gamepad gamepad1) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
	}
	@Override
	public void startAction() {
	}
	@Override
	public void runAction() {
		robot.driveTrain.robotRelative(0,0,gamepad1.right_stick_x);
		double heading = robot.getRobotPose().getAngleRadians();
		double deltaHeading = heading - lastHeading;
		headingAccumulator += Angle.normDelta(deltaHeading);
		lastHeading = heading;
		Dashboard.packet.put("Effective Odom Track Width", (headingAccumulator / (NUM_TURNS * Math.PI * 2)) * ThreeWheelOdometry.trackWidth);
	}
	@Override
	public void stopAction() {
	}
	@Override
	public boolean isActionComplete() {
		return false;
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

