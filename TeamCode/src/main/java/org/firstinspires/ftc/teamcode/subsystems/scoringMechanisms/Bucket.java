package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class Bucket implements subsystem {

	protected double lowPassFilterGain = 0.999; // 0 < x < 1
	protected LowPassFilter filter = new LowPassFilter(lowPassFilterGain);
	protected Servo bucketServo;
	DistanceSensor proximitySensor;
	String proximitySensorName = "EyeOfTheBucket";
	protected double minDistance = 1.5;
	boolean holdDown = false;



	protected Deposit.depositStates state = Deposit.depositStates.IN;
	protected Deposit.depositStates previousState = Deposit.depositStates.IN;

	double lastPosition = 1000;

	double IN = 0;
	double OUT = 1;

	protected boolean checkSensor = true;

	protected boolean isFreightInBox = false;

	double TIME_FOR_INTAKE_TO_DO_ITS_THING = 0.25;


	ElapsedTime timer = new ElapsedTime();



	@Override
	public void init(HardwareMap hwmap) {
		bucketServo = hwmap.get(Servo.class, "bucket");
		proximitySensor = hwmap.get(DistanceSensor.class, proximitySensorName);

		setPosition(IN);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}


	/**
	 * called periodically, acts as a state management system for our subsystems logic.
	 */
	@Override
	public void update() {
		//Dashboard.packet.put("BucketSensor", proximitySensor.getDistance(DistanceUnit.INCH));
		if (state.equals(Deposit.depositStates.COLLECTION) && !previousState.equals(Deposit.depositStates.COLLECTION)) {
			timer.reset();
		}


		if (state.equals(Deposit.depositStates.DEPOSITING)
				|| (state.equals(Deposit.depositStates.COLLECTION)
				&& timer.seconds() > TIME_FOR_INTAKE_TO_DO_ITS_THING)) {
			setPosition(OUT);
		} else {
			setPosition(IN);
		}
//		if (checkSensor) {
//			this.isFreightInBox = proximitySensor.getDistance(DistanceUnit.INCH) < minDistance;
//		}

		previousState = state;

	}



	@Override
	public Object subsystemState() {
		return null;
	}

	/**
	 * set the servo position using lynx optimized servo calls
	 *
	 * @param position servo position
	 */
	protected void setPosition(double position) {

		if (position != lastPosition) {
			bucketServo.setPosition(position);
		}

		lastPosition = position;
	}


	/**
	 * because it is intended for an external subroutine to set the position, this state management method is required
	 *
	 * @param state the state we would like to go to
	 */
	public void setState(Deposit.depositStates state) {
		this.state = state;
	}


	public void setCheckSensor(boolean checkSensor) {
		this.checkSensor = checkSensor;
	}

	public boolean willCheckSensor() {
		return checkSensor;
	}


	public boolean isFreightInBox() {
		if (!checkSensor) return false;
		return isFreightInBox;
	}
}
