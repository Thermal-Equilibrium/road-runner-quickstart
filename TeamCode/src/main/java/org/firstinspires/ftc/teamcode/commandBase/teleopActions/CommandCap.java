package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.gamepadEnhancements.ButtonPress;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class CommandCap implements teleopAction {

	Robot robot;

	Gamepad gamepad1;
	Gamepad gamepad2;

	ButtonPress press = new ButtonPress();


	public CommandCap(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		press.button(gamepad1.cross);
		if (press.press()) {
			robot.cappingDevice.incrementState();
		}

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
}
