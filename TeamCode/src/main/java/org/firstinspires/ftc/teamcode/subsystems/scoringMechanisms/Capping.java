package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class Capping implements subsystem {


	Servo arm;

	double previousServoPos = 10;

	protected cappingStates state = cappingStates.RESTING;

	protected LowPassFilter filter = new LowPassFilter(0.59);

	double restingPos = 1;
	double initialExtension = restingPos;
	double pickUpPosLower = 0.11;
	double pickupPos = .2;

	double aboveCapPos = .53;
	double cappedPos = 0.35;


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
			case PICKUP_LOWER:
				setServo(pickUpPosLower);
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
		double nu = filter.updateEstimate(position);

		if (nu != previousServoPos) {
			arm.setPosition(nu);
		}
		previousServoPos = nu;
	}

	public enum cappingStates {
		RESTING,
		INITIAL_EXTENSION,
		PICKUP_LOWER,
		PICKUP,
		ABOVE_CAP,
		CAPPED;

		public cappingStates next() {
			switch (this) {
				case RESTING:
					return INITIAL_EXTENSION;
				case INITIAL_EXTENSION:
					return PICKUP_LOWER;
				case PICKUP_LOWER:
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
