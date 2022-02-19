package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
@Disabled
public class TuneSlideAuto extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {
		actions.add(new GoToHighDeposit(robot));
	}
}
