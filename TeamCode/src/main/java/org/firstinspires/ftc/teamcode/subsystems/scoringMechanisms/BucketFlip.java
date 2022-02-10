package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class BucketFlip implements subsystem {

	String servoLName = "BucketLeft";
	String servoRName = "BucketRight";

	Servo leftServo;
	Servo rightServo;

	public final double COLLECTION = 0.17;
	public final double OUT = 1;
	public final double REST = 0.3;

	protected double previousPosition = 100;

	Deposit.depositStates state = Deposit.depositStates.IN;
	Deposit.depositStates previousState = Deposit.depositStates.IN;

	ElapsedTime timer = new ElapsedTime();

	double FINISH_COLLECTING_GO_UP_TIME_Seconds = 0.2;

	@Override
	public void init(HardwareMap hwmap) {

		leftServo = hwmap.get(Servo.class, servoLName);
		rightServo = hwmap.get(Servo.class, servoRName);

		rightServo.setDirection(Servo.Direction.REVERSE);
		leftServo.setDirection(Servo.Direction.FORWARD);

		//setServoPosition(OUT);


	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {

		if (state.equals(Deposit.depositStates.IN) &&
				previousState.equals(Deposit.depositStates.COLLECTION)) {
			timer.reset();
		}

		switch (state) {
			case AT_LOWEST:
				setServoPosition(OUT);

				break;
			case DISARMED:
			case IN:
			case GOING_TO_MID:
			case GOING_TO_HIGH:
			case GOING_IN:
				if (timer.seconds() > FINISH_COLLECTING_GO_UP_TIME_Seconds) {
					setServoPosition(REST);
				}
				break;
			case COLLECTION:
				setServoPosition(COLLECTION);
				break;
			case GOING_TO_LOW:
			case GOING_TO_LOW_2:
			case DEPOSITING:
			case AT_LOW:
			case AT_MID:
			case AT_HIGH:
				setServoPosition(OUT);
				break;
			default:
				// do nothing
				break;
		}

		previousState = state;


	}

	@Override
	public Deposit.depositStates subsystemState() {
		return state;
	}

	public void setState(Deposit.depositStates state) {
		this.state = state;
	}

	protected void setServoPosition(double position) {
		if (position != previousPosition) {
			this.leftServo.setPosition(position);
			this.rightServo.setPosition(position);
		}
		previousPosition = position;
	}

}
