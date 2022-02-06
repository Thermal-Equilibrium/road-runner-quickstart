package org.firstinspires.ftc.teamcode.opmodes.tutorial;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidTutorial extends LinearOpMode {

	DcMotorEx FrontLeft;
	DcMotorEx FrontRight;
	DcMotorEx BackLeft;
	DcMotorEx BackRight;


	private BNO055IMU imu;

	double integralSum = 0;
	double Kp = 0;
	double Ki = 0;
	double Kd = 0;

	ElapsedTime timer = new ElapsedTime();
	private double lastError = 0;

	@Override
	public void runOpMode() {

		FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
		FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
		BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
		BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);

		waitForStart();

		double referenceAngle = Math.toRadians(90);
		while (opModeIsActive()) {

			double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
			FrontLeft.setPower(power);
			BackLeft.setPower(power);
			FrontRight.setPower(-power);
			BackRight.setPower(-power);

		}
	}

	public double PIDControl(double reference, double state) {
		double error = angleWrap(reference - state);
		integralSum += error * timer.seconds();
		double derivative = (error - lastError) / timer.seconds();
		lastError = error;

		timer.reset();

		return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
	}

	public double angleWrap(double radians) {
		while (radians > Math.PI) {
			radians -= 2 * Math.PI;
		}
		while (radians < -Math.PI) {
			radians += 2 * Math.PI;
		}
		return radians;
	}

}
