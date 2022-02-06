package homeostasis.Filters;


import com.acmerobotics.dashboard.config.Config;

/**
 * kalman filter for single input single output state estimation
 */
@Config
public class SISOKalmanFilter {

	public static double Q = 4;
	public static double R = 0.2;
	protected double x = 0; // your initial state
	protected double p = 1; // your initial covariance guess
	protected double K = 1; // your initial kalman gain guess

	private double x_previous = x;
	private double p_previous = p;
	protected double u = 0;
	protected double z = 0;


	public SISOKalmanFilter() {

	}



	/**
	 * initialize kalman filter with covariance measurements
	 * @param Q primary sensor covariance (model covariance)
	 * @param R secondary sensor covariance
	 */
	public SISOKalmanFilter(double Q, double R) {
		this.Q = Q;
		this.R = R;
	}

	/**
	 * initialize kalman filter with covariance measurements and initial condition for state
	 * @param Q primary sensor covariance (model covariance)
	 * @param R secondary sensor covariance
	 * @param initialState the initial state of our plant
	 */
	public SISOKalmanFilter(double Q, double R, double initialState) {
		this.Q = Q;
		this.R = R;
		this.x = initialState;
	}

	/**
	 * update kalman filter
	 *
	 * INPUT MUST BE CHANGE IN SENSOR, NOT THE SENSOR VALUE,IF YOU DESIRE THIS USE THE METHOD BELOW.
	 *
	 * @param sensor1 primary sensor delta
	 * @param sensor2 secondary sensor delta
	 * @return estimate of the systems state
	 */
	public double updateKalmanFilter(double sensor1, double sensor2) {

		u = sensor1; // however you want to do this (IE, taking delta of encoder)
		x = x_previous + u;

		p = p_previous + Q;

		K = p/(p + R);

		z = sensor2; // you are probably already using a sensor for u,
		// use another sensor for z
		x = x + K * (z - x);

		p = 1 - K * p;

		x_previous = x;
		p_previous = p;

		return x;

	}

	/**
	 * update kalman filter
	 *
	 * INPUT MUST BE SENSOR VALUE, NOT THE SENSOR VALUE DELTA,IF YOU DESIRE THIS USE THE METHOD ABOVE.
	 *
	 * @param sensor1 primary sensor delta
	 * @param sensor2 secondary sensor delta
	 * @return estimate of the systems state
	 */
	public double updateKalmanMeasurements(double sensor1, double sensor2) {
		x = sensor1;

		p = p_previous + Q;

		K = p/(p + R);

		z = sensor2;
		x = x + K * (z - x);

		p = 1-K * p;

		x_previous = x;
		p_previous = p;

		return x;
	}
	/**
	 * set both covariance values at the same time
	 *
	 * Useful for systems where your trust in the sensors can vary in known conditions
	 *
	 * @param Q primary sensor covariance (model covariance)
	 * @param R secondary sensor covariance
	 */
	public void setCovariance(double Q, double R) {
		setQ(Q);
		setR(R);
	}

	/**
	 * Set primary sensor covariance
	 * @param Q primary sensor covariance (model covariance)
	 */
	public void setQ(double Q) {
		this.Q = Q;
	}

	/**
	 * set secondary sensor covariance
	 * @param R secondary sensor covariance
	 */
	public void setR(double R) {
		this.R = R;
	}


}
