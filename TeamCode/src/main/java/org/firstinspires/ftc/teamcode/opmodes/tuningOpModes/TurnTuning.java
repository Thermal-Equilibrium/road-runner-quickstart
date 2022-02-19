package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Disabled
public class TurnTuning extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {
		actions.add(new Turn(robot, Math.toRadians(90)));
	}
}
