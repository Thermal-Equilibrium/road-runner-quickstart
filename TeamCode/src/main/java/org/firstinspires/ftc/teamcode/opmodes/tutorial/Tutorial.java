package org.firstinspires.ftc.teamcode.opmodes.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Tutorial extends LinearOpMode {

	// motor declaration, we use the
	// Ex version as it has velocity measurements
	DcMotorEx motor;
	// create our PID controller
	PIDController control = new PIDController(0.05, 0, 0);

	@Override
	public void runOpMode() {
		// the string is the hardware map name
		motor = hardwareMap.get(DcMotorEx.class, "arm");

		// use braking to slow the motor down faster
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// disables the default velocity control
		// this does NOT disable the encoder from counting,
		// but lets us simply send raw motor power.
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		waitForStart();
		// loop that runs while the program should run.

		int targetPosition = 100;

		while (opModeIsActive()) {
			// update pid controller
			double command = control.update(targetPosition, motor.getVelocity());
			// assign motor the PID output
			motor.setPower(command);
		}
	}
}


