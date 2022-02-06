package org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;

/**
 * Nonblocking delay for command base
 */
public class Delay implements action {

	protected boolean isComplete = false;
	double durationMs;
	ElapsedTime timer = new ElapsedTime();

	public Delay(double durationMs) {
		this.durationMs = durationMs;
	}

	@Override
	public void startAction() {
		timer.reset();
	}

	@Override
	public void runAction() {

		if (timer.milliseconds() > durationMs) {
			isComplete = true;
		}

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
