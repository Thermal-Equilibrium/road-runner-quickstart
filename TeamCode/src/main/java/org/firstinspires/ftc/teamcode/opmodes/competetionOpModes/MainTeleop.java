package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.teleopActions.CommandCap;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.CommandDeposit2;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.Drive.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.GoToCollectionState;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.RezeroHeading;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleBox;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleDuckWheel2;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleIntake;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseTeleop;


@TeleOp
public class MainTeleop extends BaseTeleop {
	@Override
	public void addActions() {
		actions.add(new FieldRelativeDrive(robot,gamepad1,gamepad2));
		actions.add(new ToggleIntake(robot, gamepad1, gamepad2));
		actions.add(new CommandDeposit2(robot, gamepad1, gamepad2));
		actions.add(new ToggleDuckWheel2(robot,gamepad1,gamepad2));
		actions.add(new RezeroHeading(robot,gamepad1,gamepad2));
		actions.add(new CommandCap(robot,gamepad1,gamepad2));
		actions.add(new ToggleBox(robot, gamepad1, gamepad2)); // this must go last or there will be a race condition
	}
}


