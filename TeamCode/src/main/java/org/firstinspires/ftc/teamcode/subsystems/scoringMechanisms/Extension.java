package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class Extension implements subsystem {

	protected final double COLLECTION = 1;
	protected final double IN = 1;
	public final double LOW_1 = 0.77;
	public final double LOW_2 = 0.7;
	public double LOW = LOW_1;
	protected final double MID = 0.6;
	protected final double HIGH = 0.4;
	protected Servo extension;
	protected Deposit.depositStates state = Deposit.depositStates.IN;
	protected double lastPosition = 1000;

	ElapsedTime timer = new ElapsedTime();

	@Override
	public void init(HardwareMap hwmap) {
		extension = hwmap.get(Servo.class, "extension");

		setPosition(IN);

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {

		switch (state) {
			case DISARMED:
				setPosition(IN);
				System.out.println("disarmed");
				break;
			case COLLECTION:
				setPosition(COLLECTION);
				break;
			case AT_LOWEST:
			case IN:
			case GOING_TO_LOW:
			case GOING_TO_MID:
			case GOING_TO_HIGH:
			case GOING_IN:
			case GOING_TO_LOW_2:
				setPosition(IN);
				break;
			case AT_HIGH:
				setPosition(HIGH);
				break;
			case AT_MID:
				setPosition(MID);
				break;
			case AT_LOW:
				setPosition(LOW);
				break;
			case DEPOSITING:
				// theoretically nothing should change?
				break;
			case CAP_RESTING:
				break;
			case CAP_INITIAL_EXTENSION:
				setPosition(HIGH);
				break;
			case CAP_PICKUP:
				setPosition(HIGH);
				break;
			case CAP_ABOVE_CAP:
				setPosition(HIGH);
				break;
			case CAP_CAPPED:
				setPosition(HIGH);
				break;
		}
		timer.reset();

	}

	@Override
	public Object subsystemState() {
		return null;
	}


	public void setState(Deposit.depositStates state) {
		this.state = state;
	}

	/**
	 * set the 4 bar angle while optimizing lynx calls
	 *
	 * @param position the -1 < theta < 1 angle of the virtual 4 bar
	 */
	protected void setPosition(double position) {

		if (position != lastPosition) {
			extension.setPosition(position);
		}

		lastPosition = position;
	}
}
