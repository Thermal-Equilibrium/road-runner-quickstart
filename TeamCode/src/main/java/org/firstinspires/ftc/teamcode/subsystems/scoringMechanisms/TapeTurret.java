package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class TapeTurret implements subsystem {

	protected Robot robot;
	CRServo tape;
	Servo turret;
	protected final double TURRET_INITIAL_POSITION = 0.5;
	protected double previous_tape_power = 10;
	protected double previous_turret_position = 0;

	protected double turretPos = 0.5;
	protected double tapeSpeed = 0;


	@Override
	public void init(HardwareMap hwmap) {
		tape = hwmap.get(CRServo.class, "tape");
		turret = hwmap.get(Servo.class, "turret");
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {
		setTurret(turretPos, tapeSpeed);
	}

	@Override
	public Object subsystemState() {
		return null;
	}

	public void setTurret(double turretPos, double tapePower) {
		if (turretPos != previous_turret_position) {
			turret.setPosition(turretPos);
		}
		if (tapePower != previous_tape_power) {
			tape.setPower(tapePower);
		}
		previous_tape_power = tapePower;
		previous_turret_position = turretPos;
	}

	public void setTapeSpeed(double tapeSpeed) {
		this.tapeSpeed = tapeSpeed;
	}

	public void setTurretPos(double turretPos) {
		this.turretPos = turretPos;
	}
}
