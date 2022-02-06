package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.ON;
import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.OTHER_ON;

public class ToggleDuckWheel implements teleopAction {

	Robot robot;
	Gamepad gamepad1;
	Gamepad gamepad2;

	public ToggleDuckWheel(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

		@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		if (gamepad1.dpad_down) robot.duckwheel.setState(ON);
		else if (gamepad1.dpad_left) robot.duckwheel.setState(OTHER_ON);
		else robot.duckwheel.setState(OFF);
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
