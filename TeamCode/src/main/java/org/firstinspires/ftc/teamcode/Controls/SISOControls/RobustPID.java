package org.firstinspires.ftc.teamcode.Controls.SISOControls;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.Coefficients.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Controls.utils.PIDState;
import org.firstinspires.ftc.teamcode.Utils.RingBuffer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.Utils.utils.normalizedHeadingError;

public class RobustPID {

	// coefficients of the controller
	protected PIDFCoefficients coefficients;

	protected final double EPSILON = 1e-2;

	// elapsed timer used for integral and derivative calculations
	protected ElapsedTime timer;

	private final RingBuffer<PIDState> derivativeBuffer; // good morning bfr <3

	// process error
	protected double error = 0;
	// previous error
	protected double lastError = 0;
	// cumulative error sum over time
	protected double integral_sum = 0;
	// error rate of change
	protected double derivative = 0;
	// output that we send to our plant
	protected double output = 0;
	// delta time between updates
	protected double dt = 0;
	// previous time stamp (seconds)
	protected double lastTime;
	// current time stamp (seconds)
	protected double time;
	// where our plants state should be
	protected double reference;
	// how long the RingBuffers length should store data for
	protected int bufferLength;
	// exit tolerance
	protected double exitTolerance;
	// if the derivative is above this, we are not considering the system stable
	protected double stability_threshold;
	// previous feedback output for adaptive feedforward control
	protected double previous_feedback = 0;
	protected boolean limitIntegralSum = true;

	/**
	 * construct PID with buffer length and stability threshold
	 *
	 * @param coefficients        pid coefficients
	 * @param reference           the target state our system should be in
	 * @param bufferLength        how long our buffer should be
	 * @param stability_threshold how stable our derivative needs to be inorder to stop
	 */
	public RobustPID(PIDFCoefficients coefficients, double reference, int bufferLength, double stability_threshold, double exitTolerance) {
		this.reference = reference;
		this.bufferLength = bufferLength;
		this.stability_threshold = stability_threshold;
		this.coefficients = coefficients;
		this.exitTolerance = exitTolerance;
		timer = new ElapsedTime();
		lastTime = timer.time();
		PIDState PIDFData = new PIDState(lastError, lastTime);
		derivativeBuffer = new RingBuffer<>(bufferLength, PIDFData);
	}

	public RobustPID(PIDFCoefficients coefficients, int bufferLength, double stability_threshold, double exitTolerance) {
		this.reference = 0;
		this.bufferLength = bufferLength;
		this.stability_threshold = stability_threshold;
		this.coefficients = coefficients;
		this.exitTolerance = exitTolerance;
		timer = new ElapsedTime();
		lastTime = timer.time();
		PIDState PIDFData = new PIDState(lastError, lastTime);
		derivativeBuffer = new RingBuffer<>(bufferLength, PIDFData);

	}

	/**
	 * perform general PID calculations and operations
	 */
	protected void baseCalculate() {
		updateTime();
		calculateDerivative();
		calculateIntegral();
		double out1 = (error * coefficients.Kp) + (integral_sum * coefficients.Ki);

		if (Math.abs(out1) < coefficients.H && Math.abs(error) > EPSILON) {
			out1 = coefficients.H * Math.signum(out1);
		}

		output = out1 +
				(reference * coefficients.Kf) +
				(derivative * coefficients.Kd);
		previous_feedback = output;
		output += adaptiveFeedforward();
		lastError = error;
	}


	/**
	 * calculate pid output with normal linear values
	 *
	 * @param state current measurement of our system
	 * @return input to the plant
	 */
	public double calculate(double state) {
		calculateErrorNormal(state);
		baseCalculate();
		return output;
	}

	public double stateReferenceCalculate(double reference, double state) {
		this.reference = reference;
		calculateErrorNormal(state);
		baseCalculate();
		return output;
	}


	/**
	 * calculate pid output with angle values
	 *
	 * @param state current measurement of our system (radians)
	 * @return input to the plant
	 */
	public double calculateAngle(double state) {
		calculateErrorAngle(state);
		baseCalculate();
		output = nonlinearAngleControl() + (derivative * coefficients.Kd) + (integral_sum * coefficients.Ki);
		return output;
	}
	/**
	 * calculate pid output with angle values
	 * @param state current measurement of our system (radians)
	 * @return input to the plant
	 */
	public double calculateLinearAngle(double state) {
		calculateErrorAngle(state);
		baseCalculate();
		return output;
	}

	/**
	 * calculate derivative using the buffer to smooth the data
	 */
	protected void calculateDerivative() {
		PIDState bufferedPoint = derivativeBuffer.insert(new PIDState(error, time));

		derivative = (error - bufferedPoint.error) / (time - bufferedPoint.timeStamp);
	}

	/**
	 * calculate integral sum given our system is stable
	 */
	protected void calculateIntegral() {
		double avg = (error + lastError) / 2;

		if (shouldIntegrate()) {
			integral_sum += avg * dt;
		}
		antiWindup();

	}

	/**
	 * assess if our system is stable by checking the derivative
	 * @return true if stable, false if not
	 */
	public boolean isStable() {
		return Math.abs(stability_threshold) > Math.abs(derivative);
	}
	public boolean isVeryStable() {
		return Math.abs(stability_threshold / 5) > Math.abs(derivative);
	}

	public boolean isBasicallyStopped() {
		return Math.abs(derivative) < 1e-4;
	}
	/**
	 * conditions for if we should integrate
	 * @return true if we should integrate
	 */
	public boolean shouldIntegrate() {
		return true;
	}

	/**
	 * update time step
	 */
	protected void updateTime() {
		time = timer.seconds();
		dt = time - lastTime;
		lastTime = time;
	}

	/**
	 * calculate linear error
	 * @param state systems state
	 */
	protected void calculateErrorNormal(double state) {
		error = reference - state;
	}

	/**
	 * calculate angle error
	 * @param state systems state
	 */
	protected void calculateErrorAngle(double state) {
		error = normalizedHeadingError(reference, state);
	}

	/**
	 * return the current plant error
	 *
	 * @return error
	 */
	public double getError() {
		return error;
	}

	/**
	 * an improved feedback controller designed specifically for angle control
	 * <p>
	 * The idea is to use two exponential functions as a piecewise function to generate our command input
	 * <p>
	 * the function Kp^e(t) - H is our controllers feedback calculation
	 * <p>
	 * This method allows us to better approximate the static friction of our drivetrain and compensate for it
	 * <p>
	 * stable under the https://en.wikipedia.org/wiki/Lyapunov_stability assuming friction is apart of the equation
	 * <p>
	 * hysteresis amount is the percentage relative to 0 that we go over, such as 96 percent would be 4% over the minimum motor power
	 * <p>
	 * The idea with the hysteresis is that a minimum power will act as a feed forward to counter our drivetrains static friction
	 *
	 * @return the control input for angles
	 */
	protected double nonlinearAngleControl() {
		double hysteresisAmount = 0.96;

		if (Robot.isCompBot) {
			hysteresisAmount = 0.80;
		}
		if (error > 0) {
			return Math.pow(coefficients.Kp, error) - hysteresisAmount;
		}
		return -Math.pow(coefficients.Kp, -error) + hysteresisAmount;
	}

	public boolean isComplete() {
		return Math.abs(error) < exitTolerance && isStable();
	}

	public void antiWindup() {

		if (Math.signum(error) != Math.signum(integral_sum)) {
			integral_sum = 0;
		}


		if (!limitIntegralSum) return;
		if ((lastError > 0 && error < 0) || ((lastError < 0 && error > 0))) {
			integral_sum = 0;
		}
		if (integral_sum > 1) integral_sum = 1;
		if (integral_sum < -1) integral_sum = -1;



	}


	public void setLimitIntegralSum(boolean limitIntegralSum) {
		this.limitIntegralSum = limitIntegralSum;
	}

	/**
	 * change the reference (imo)
	 *
	 * @param reference reference position
	 */
	public void setReference(double reference) {
		this.reference = reference;
	}

	public double getReference() {
		return reference;
	}


	public double adaptiveFeedforward() {
		return previous_feedback * (0.25 * coefficients.Kp);
	}


}
