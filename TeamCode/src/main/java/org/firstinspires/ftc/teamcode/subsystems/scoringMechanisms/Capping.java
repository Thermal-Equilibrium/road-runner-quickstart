package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class Capping implements subsystem {


	Servo arm;

	double previousServoPos = 10;

	protected cappingStates state = cappingStates.RESTING;

	double restingPos = 1;
	double initialExtension = restingPos;
	double pickupPos = .1;
	double aboveCapPos = .5;
	double cappedPos = .4;


	@Override
	public void init(HardwareMap hwmap) {
		arm = hwmap.get(Servo.class, "Cap arm");
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {
		switch (state) {
			case RESTING:
				setServo(restingPos);
				break;
			case INITIAL_EXTENSION:
				setServo(initialExtension);
				break;
			case PICKUP:
				setServo(pickupPos);
				break;
			case ABOVE_CAP:
				setServo(aboveCapPos);
				break;
			case CAPPED:
				setServo(cappedPos);
				break;
		}
	}

	@Override
	public cappingStates subsystemState() {
		return state;
	}

	public void setState(cappingStates state) {
		this.state = state;
	}

	public void incrementState() {
		this.state = this.state.next();
	}

	public void decrementState() {
		this.state = this.state.previous();
	}

	protected void setServo(double position) {
		if (position != previousServoPos) {
			arm.setPosition(position);
		}
		previousServoPos = position;
	}

	public enum cappingStates {
		RESTING,
		INITIAL_EXTENSION,
		PICKUP,
		ABOVE_CAP,
		CAPPED;

		public cappingStates next() {
			switch (this) {
				case RESTING:
					return INITIAL_EXTENSION;
				case INITIAL_EXTENSION:
					return PICKUP;
				case PICKUP:
					return ABOVE_CAP;
				case ABOVE_CAP:
					return CAPPED;
				case CAPPED:
				default:
					return RESTING;
			}
		}

		public cappingStates previous() {
			switch (this) {
				case ABOVE_CAP:
					return PICKUP;
				case CAPPED:
					return ABOVE_CAP;
				case RESTING:
				case PICKUP:
				default:
					return RESTING;
			}
		}

	}
}
