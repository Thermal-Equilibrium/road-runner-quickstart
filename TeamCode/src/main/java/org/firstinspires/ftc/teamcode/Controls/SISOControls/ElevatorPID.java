package org.firstinspires.ftc.teamcode.Controls.SISOControls;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ElevatorPID {
	private final double kp;
	private final double ki;
	private final double kd;
	private final double kg;
	private final double a;
	public double Kp; // proportional coefficient
	public double Ki; // integral coefficient
	public double Kd; // derivative coefficient
	public double Kg; // gravity constant
	public double A; // low pass filter gain, smooths derivative

	protected double previousError = 0;
	protected double integralSum = 0;
	protected double previousLowPassEstimate = 0;

	protected boolean hasBegun = false;

	protected ElapsedTime timer = new ElapsedTime();

	/**
	 * construct PID controller for elevator type system.
	 * @param Kp traditional proportional coefficient
	 * @param Ki traditional integral coefficient
	 * @param Kd traditional derivative coefficient
	 * @param Kg Gravity constant
	 * @param A filter coefficient for derivative noise reduction (0 < a < 1)
	 */
	public ElevatorPID(double Kp, double Ki, double Kd, double Kg, double A) {
		kp = Kp;
		ki = Ki;
		kd = Kd;
		kg = Kg;
		a = A;
	}

	/**
	 *
	 * @param reference the target position we would like to reach
	 * @param state the current position we are in
	 * @return the current motor command.
	 */
	public double update(double reference, double state) {

		double dt = getDt();
		double error = reference - state;
		integralSum = calculateIntegral(error, integralSum, dt);
		double derivative = calculateDerivative(error, previousError, dt);
		return error * Kp + derivative * Kd + integralSum * Ki + Kg;

	}

	/**
	 * calculate integral sum
	 * @param error current error
	 * @param integral_sum current integral sum
	 * @param dt time constant
	 * @return new integral sum
	 */
	protected double calculateIntegral(double error, double integral_sum, double dt) {
		return integral_sum + error * dt;
	}

	/**
	 * calculate the derivative
	 * @param error current error
	 * @param previousError previous error
	 * @param dt time constant
	 * @return derivative estimate
	 */
	protected double calculateDerivative(double error, double previousError, double dt) {
		return lowPassFilter((error - previousError) / dt);
	}

	/**
	 * get the current time constant in seconds
	 * @return time constant in seconds
	 */
	protected double getDt() {
		if (!hasBegun) {
			timer.reset();
			hasBegun = true;
		}
		double dt = timer.seconds();
		timer.reset();
		return dt;
	}

	/**
	 * low pass filter for derivative
	 * @param derivative the current naive derivative calculation
	 * @return the filtered derivative
	 */
	protected double lowPassFilter(double derivative) {
		double estimate = a * previousLowPassEstimate + (1 - a) * derivative;
		previousLowPassEstimate = estimate;
		return estimate;
	}

}
