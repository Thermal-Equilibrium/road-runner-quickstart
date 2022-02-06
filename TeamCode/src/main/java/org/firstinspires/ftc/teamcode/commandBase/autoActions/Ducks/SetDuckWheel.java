package org.firstinspires.ftc.teamcode.commandBase.autoActions.Ducks;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SetDuckWheel implements action {

    Robot robot;
    DuckWheel.DuckWheelState targetState;

    public boolean isComplete = false;
    public SetDuckWheel(Robot robot, DuckWheel.DuckWheelState targetState) {
        this.robot = robot;
        this.targetState = targetState;
    }

    @Override
    public void startAction() {
        robot.duckwheel.setState(targetState);
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
