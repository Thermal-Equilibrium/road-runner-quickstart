package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepadEnhancements.ButtonPress;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.ON;
import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.ON_FAST;
import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.OTHER_ON;
import static org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheelState.OTHER_ON_FAST;

public class ToggleDuckWheel2 extends ToggleDuckWheel{


	ElapsedTime timer = new ElapsedTime();

	ButtonPress downPress = new ButtonPress();
	ButtonPress leftPress  = new ButtonPress();

	public final double TIME_TILL_SPEED = 1; // time in seconds until it starts going zoom.

	public ToggleDuckWheel2(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		super(robot, gamepad1, gamepad2);
		timer.reset();
	}

	@Override
	public void periodic() {
		boolean down = gamepad1.dpad_down;
		boolean left = gamepad1.dpad_left;

		downPress.button(down);
		leftPress.button(left);

		if (downPress.press() || leftPress.press()) {
			timer.reset();
		}

		double time = timer.seconds();

		if (down) {
			if (time >= TIME_TILL_SPEED) {
				robot.duckwheel.setState(ON_FAST);
			} else {
				robot.duckwheel.setState(ON);
			}
		} else if (left) {
			if (time >= TIME_TILL_SPEED) {
				robot.duckwheel.setState(OTHER_ON_FAST);
			} else {
				robot.duckwheel.setState(OTHER_ON);
			}
		} else {
			robot.duckwheel.setState(OFF);
			timer.reset();
		}
	}
}
