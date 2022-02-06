package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckWheel implements subsystem{

	protected CRServo Duck;

	protected DuckWheelState state = DuckWheelState.OFF;

	protected double previousPower = 0;
	protected final double TURN_POWER = -1;
	protected final double FAST_TURN_POWER = -1;

	@Override
	public void init(HardwareMap hwmap) {
		this.Duck = hwmap.get(CRServo.class, "Duck");
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {
		switch (state) {
			case ON:
				setServoPower(TURN_POWER);
				break;
			case OTHER_ON:
				setServoPower(-TURN_POWER);
				break;
			case ON_FAST:
				setServoPower(FAST_TURN_POWER);
				break;
			case OTHER_ON_FAST:
				setServoPower(-FAST_TURN_POWER);
				break;
			case OFF:
				setServoPower(0);
				break;
		}
	}

	public void setState(DuckWheelState state) {
		this.state = state;
	}

	@Override
	public DuckWheelState subsystemState() {
		return state;
	}

	public void setServoPower(double power) {
		if (power != previousPower) {
			Duck.setPower(-power);
		}
		previousPower = power;
	}

	public enum DuckWheelState {
		ON,
		OTHER_ON,
		ON_FAST,
		OTHER_ON_FAST,
		OFF
	}
}
