package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import homeostasis.utils.State;


/**
 * main class for the entire deposit including bucket, slides, and virtual 4 bar
 */
public class Deposit implements subsystem {

	public double slideErrorTolerance = 5;
	protected Slides slideSystem = new Slides();
	public Extension v4b = new Extension();
	protected depositStates state = depositStates.IN;
	protected BucketFlip bucketFlip = new BucketFlip();

	@Override
	public void init(HardwareMap hwmap) {
		slideSystem.init(hwmap);
		v4b.init(hwmap);
		bucketFlip.init(hwmap);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		slideSystem.initNoReset(hwmap);
		v4b.initNoReset(hwmap);
		bucketFlip.initNoReset(hwmap);
	}

	@Override
	public void update() {

		slideSystem.setState(state);
		v4b.setState(state);
		bucketFlip.setState(state);

		slideSystem.update();
		v4b.update();
		bucketFlip.update();
	}

	public depositStates getState() {
		return state;
	}

	public void setState(depositStates state) {
		this.state = state;
	}

	/**
	 * for now just return the position of the slides
	 *
	 * @return position of the slides
	 */
	@Override
	public State subsystemState() {
		return slideSystem.subsystemState();
	}

	/**
	 * assess if our linear slides are close enough to the reference to continue
	 *
	 * @return true if we are within tolerance to the reference
	 */
	public boolean isSlideWithinTolerance() {
		return Math.abs(slideSystem.getControllerError()) < slideErrorTolerance;
	}

	public boolean tolerantEnoughForDeploy() {
		return Math.abs(slideSystem.getControllerError()) < 10;
	}

	public enum depositStates {
		DISARMED, // motor power is cut
		IN, // everything is in, ready for going over
		COLLECTION,
		GOING_TO_HIGH,
		GOING_TO_MID,
		GOING_TO_LOW,
		GOING_TO_LOW_2, // weird thing because our bucket is sus.
		AT_HIGH, // at high but not deposited
		AT_MID, // at mid but not deposited
		AT_LOW, // at low but not deposited
		DEPOSITING, //depositing
		AT_LOWEST,
		GOING_IN,
		CAP_RESTING,
		CAP_INITIAL_EXTENSION,
		CAP_PICKUP,
		CAP_ABOVE_CAP,
		CAP_CAPPED
	}
}

