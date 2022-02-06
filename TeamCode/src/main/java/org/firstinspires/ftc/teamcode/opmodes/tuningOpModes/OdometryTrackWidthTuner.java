package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.TuneOdometryTrackWidth;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;


@TeleOp
public class OdometryTrackWidthTuner extends BaseAuto {
	@Override
	public void setStartingPosition() {
	}
	@Override
	public void setVisionSettings() {
	}
	@Override
	public void addActions() {
		actions.add(new TuneOdometryTrackWidth(robot,gamepad1));
	}
}