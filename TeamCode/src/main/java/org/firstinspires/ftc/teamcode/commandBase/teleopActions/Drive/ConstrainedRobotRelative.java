package org.firstinspires.ftc.teamcode.commandBase.teleopActions.Drive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ConstrainedRobotRelative extends NormalRobotRelative {

	public double minimumYThreshold = 0.1;

	public ConstrainedRobotRelative(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		super(robot, gamepad1, gamepad2);
	}

	@Override
	public void periodic() {
		robot.driveTrain.robotRelativeConstrained(getMotorPowers(),minimumYThreshold);
	}
}
