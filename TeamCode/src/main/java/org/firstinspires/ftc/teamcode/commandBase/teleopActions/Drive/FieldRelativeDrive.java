package org.firstinspires.ftc.teamcode.commandBase.teleopActions.Drive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FieldRelativeDrive implements teleopAction {

	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;


	public FieldRelativeDrive(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}


	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		robot.driveTrain.fieldRelative(getMotorPowers());
	}

	@Override
	public boolean isComplete() {
		return false;
	}

	@Override
	public boolean shouldRun() {
		return true;
	}

	@Override
	public void reset() {

	}

	@Override
	public boolean hasPerformedInitialRun() {
		return true;
	}

	public Vector3D getMotorPowers() {
		return new Vector3D(-gamepad1.right_stick_y,
							gamepad1.right_stick_x,
							gamepad1.left_stick_x);
	}

}
