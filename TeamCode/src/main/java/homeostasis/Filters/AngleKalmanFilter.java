package homeostasis.Filters;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Utils.utils.AngleWrap;

@Config
public class AngleKalmanFilter {

	protected double x;
	protected double p;
	protected double x_previous;
	protected double p_previous;
	public static double Q = 9.504936835155863;
	public static double R = 5.815049368351558;
	protected double currentModel;
	protected double previousModel;
	protected double currentSensor2;
	protected double previousSensor2;

	protected double A = 1;
	protected double C = 1;
	protected double H = 1;
	protected double I = 1;
	protected double kalmanGain;

	ElapsedTime timer = new ElapsedTime();

	public AngleKalmanFilter(double initialCondition) {
		this.x = initialCondition;
		this.x_previous = initialCondition;
		this.p = 0;
		this.p_previous = 0;
		this.currentModel = initialCondition;
		this.previousModel = initialCondition;
		this.currentSensor2 = initialCondition;
		this.previousSensor2 = initialCondition;

	}

	/**
	 * update the odometry measurement from our sensor1 and gyro angle
	 *
	 * @param model primary angle sensor
	 * @param sensor2 most stable angle sensor that we are converging on over time
	 * @return estimated angle from kalman filter
	 */
	public double updateKalmanEstimate(double model, double sensor2) {
		currentModel = model;
		currentSensor2 = sensor2;
		previousModel = model;
		previousSensor2 = sensor2;
		x = x_previous;
		p = p_previous + Q;
		kalmanGain = p / (p + R);
		System.out.println("Kalman gain is: " + kalmanGain);
		System.out.println("Kalman x previous is " + x_previous);
		System.out.println("Kalman current sensor 2 is " + currentSensor2);
		x = x + kalmanGain * AngleWrap(currentSensor2 - x_previous);
		System.out.println("Kalman filter estimate is: " + x);

		x = AngleWrap(x);
		p = 1 - kalmanGain * p;
		x_previous = x;
		p_previous = p;
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}
}
