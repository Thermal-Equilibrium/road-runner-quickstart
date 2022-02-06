package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.COLLECTION;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.ON;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.REVERSE;

public class ToggleIntake implements teleopAction {


	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;


	public ToggleIntake(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {

		if (robot.Deposit.getState().equals(COLLECTION) && gamepad1.right_trigger > 0.5) {
			robot.Intake.setState(ON);
		} else if (gamepad1.left_trigger > 0.5) {
			robot.Intake.setState(REVERSE);
		} else {
			robot.Intake.setState(OFF);
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
