package homeostasis.Filters;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;

/**
 *
 * using a kalman filter, more accurately obtain velocity measurements of systems
 *
 */

public class VelocityKalmanFilter extends SISOKalmanFilter {

	protected DcMotorEx motor;
	protected double previousPosition = 0;
	protected ElapsedTime timer = new ElapsedTime();
	protected LowPassFilter filter = new LowPassFilter(0.9);

	/**
	 * initialize filter and give access to our motor
	 * @param motor the motor we want to estimate the velocity of
	 */
	public VelocityKalmanFilter(DcMotorEx motor) {
		super();
		this.motor = motor;
	}

	public double estimateVelocity() {

		double position = motor.getCurrentPosition();
		double derivedVelocity = (position - previousPosition) / timer.seconds();
		previousPosition = position;
		double quantized = motor.getVelocity();
		double velocityEstimate = updateKalmanMeasurements(derivedVelocity,quantized);

		timer.reset();

		Dashboard.packet.put("d",derivedVelocity);
		Dashboard.packet.put("q",quantized);
		Dashboard.packet.put("e",velocityEstimate);
		System.out.println("quantized: " + quantized + " derived: " + derivedVelocity + " estimated " + velocityEstimate);

		return quantized; // TODO change this back to the velocity estimate once the kalman filter works

	}

	public double lowPassVelocity() {
		double raw = motor.getVelocity();

		return filter.updateEstimate(raw);

	}


}
