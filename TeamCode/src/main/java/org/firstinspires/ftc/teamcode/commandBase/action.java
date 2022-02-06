package org.firstinspires.ftc.teamcode.commandBase;

public interface action {

	void startAction();

	void runAction();

	void stopAction();

	boolean isActionComplete();

	boolean isActionPersistent();

	boolean isAMultipleAction();



}


