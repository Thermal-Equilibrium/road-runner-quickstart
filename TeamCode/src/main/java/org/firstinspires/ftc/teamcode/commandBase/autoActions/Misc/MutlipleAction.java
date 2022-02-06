package org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc;

import org.firstinspires.ftc.teamcode.commandBase.action;

public class MutlipleAction implements action {

    action[] actions;

    public MutlipleAction(action[] actions) {
        this.actions = actions;
    }

    @Override
    public void startAction() {

        for (action a : actions) {
            a.startAction();
        }

    }

    @Override
    public void runAction() {
        for (action a : actions) {
            a.runAction();
        }
    }

    @Override
    public void stopAction() {

        for (action a : actions) {
            a.stopAction();
        }

    }

    @Override
    public boolean isActionComplete() {

        for (action a : actions) {
            if (!a.isActionComplete()) return false;
        }

        return true;
    }

    @Override
    public boolean isActionPersistent() {
        for (action a : actions) {
            if (a.isActionComplete()) return true;
        }

        return false;
    }

    @Override
    public boolean isAMultipleAction() {
        return true;
    }
}
