package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeadwheelRetract implements subsystem{

	Servo odo1 ;
	Servo odo2 ;
	double upPosition = .5;
	double downPosition = 1;




	@Override
	public void init(HardwareMap hwmap) {
		odo1 = hwmap.get(Servo.class, "odo1");
		odo2 = hwmap.get(Servo.class, "odo2");

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {

	}

	@Override
	public void update() {

	}

	@Override
	public Object subsystemState() {
		return null;
	}

	public void retract() {
		odo1.setPosition(downPosition);
		odo2.setPosition(upPosition);
	}

	public void down() {
		odo1.setPosition(upPosition);
		odo2.setPosition(downPosition);
	}
}
