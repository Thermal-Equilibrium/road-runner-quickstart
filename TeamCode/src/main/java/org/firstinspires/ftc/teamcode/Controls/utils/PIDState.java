package org.firstinspires.ftc.teamcode.Controls.utils;


/**
 * concept stolen from black forest robotics to store a previous PID controller state for improved derivative calculations
 */
public class PIDState {

	public final double error;
	public final double timeStamp;

	public PIDState(double error, double timeStamp) {
		this.error = error;
		this.timeStamp = timeStamp;
	}

	public double getError() {
		return error;
	}

	public double getTimeStamp() {
		return timeStamp;
	}

	@Override
	public String toString() {
		return "PIDState{" +
				"error=" + error +
				", timeStamp=" + timeStamp +
				'}';
	}
}
