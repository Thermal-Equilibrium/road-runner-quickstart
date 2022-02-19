package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Disabled
public class MultipleActionExample extends BaseAuto {

    action drive = new DriveToPosition(robot, new Vector3D(10,0,0));
    action putSlidesUp = new GoToHighDeposit(robot);
    @Override
    public void setStartingPosition() {

    }

    @Override
    public void setVisionSettings() {

    }

    @Override
    public void addActions() {


    }
}
