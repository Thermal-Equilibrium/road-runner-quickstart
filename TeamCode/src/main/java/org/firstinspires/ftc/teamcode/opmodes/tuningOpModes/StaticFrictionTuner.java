package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.FindStaticFrictionForward;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.FindStaticFrictionTurn;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Autonomous
public class StaticFrictionTuner extends BaseAuto {

	@Override
	public void setStartingPosition() {

	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {

		actions.add(new FindStaticFrictionForward(robot));
		actions.add(new FindStaticFrictionTurn(robot));

	}
}
