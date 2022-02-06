package org.firstinspires.ftc.teamcode.commandBase.teleopActions.Drive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class NormalRobotRelative extends FieldRelativeDrive {
	public NormalRobotRelative(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		super(robot, gamepad1, gamepad2);
	}

	@Override
	public void periodic() {
		robot.driveTrain.robotRelative(getMotorPowers());
	}

}
