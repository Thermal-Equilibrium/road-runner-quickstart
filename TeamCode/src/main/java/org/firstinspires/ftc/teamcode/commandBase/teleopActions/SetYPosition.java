package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SetYPosition implements action {

    Robot robot;
    double new_y_pos;
    protected boolean isComplete = false;

    public SetYPosition(Robot robot, double new_y_pos) {
        this.robot = robot;
        this.new_y_pos = new_y_pos;
    }

    @Override
    public void startAction() {
        Vector3D pose = robot.getRobotPose();
        pose.setY(new_y_pos);
        robot.setRobotPose(pose);
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
