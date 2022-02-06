package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class Intake implements subsystem {


	protected double lastPower = 1000;
	protected DcMotorEx intakeMotor;

	protected double ON_POWER = -1;

	protected intakeStates state = intakeStates.OFF;


	@Override
	public void init(HardwareMap hwmap) {
		intakeMotor = hwmap.get(DcMotorEx.class, "Intake");
		intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {


		double power;

		// default to ON
		switch (state) {
			case OFF:
				power = 0;
				break;
			case REVERSE:
				power = -ON_POWER;
				break;
			default:
				power = ON_POWER;
		}

		setPower(power);


	}

	@Override
	public Object subsystemState() {
		return null;
	}

	public void setState(intakeStates state) {
		this.state = state;
	}

	public void setPower(double power) {
		if (power != lastPower) {
			intakeMotor.setPower(power);
		}
		lastPower = power;
	}

	public enum intakeStates {
		ON,
		OFF,
		REVERSE
	}

}
