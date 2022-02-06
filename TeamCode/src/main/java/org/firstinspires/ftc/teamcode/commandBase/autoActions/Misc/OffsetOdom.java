package org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OffsetOdom implements action {

    Robot robot;
    Vector3D odomOffset;
    boolean isComplete = false;

    public OffsetOdom(Robot robot, Vector3D odomOffset) {
        this.robot = robot;
        this.odomOffset = odomOffset;
    }

    @Override
    public void startAction() {

        Vector3D robotPose = robot.getRobotPose();
        Vector3D newPose = robotPose.plus(odomOffset);
        robot.setRobotPose(newPose);
        isComplete = true;
    }

    @Override
    public void runAction() {

    }

    @Override
    public void stopAction() {

    }

    @Override
    public boolean isActionComplete() {
        return isComplete;
    }

    @Override
    public boolean isActionPersistent() {
        return false;
    }

    @Override
    public boolean isAMultipleAction() {
        return false;
    }
}
