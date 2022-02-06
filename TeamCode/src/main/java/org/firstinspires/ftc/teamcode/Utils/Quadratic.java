package org.firstinspires.ftc.teamcode.Utils;

public class Quadratic {

	protected double a;
	protected double b;
	protected double c;

	public Quadratic(double a,double b, double c) {
		this.a = a;
		this.b = b;
		this.c = c;
	}

	/**
	 * evaluate the value of the function at t
	 * @param t the time stamp t
	 * @return the current value
	 */
	public double evaluateFunction(double t) {
		return (a * Math.pow(t,2)) + (b * t) + c;
	}

	/**
	 * evaluate the integral from 0 to t
	 * @param t time t
	 * @return the integral sum at this point in time
	 */
	public double evaluateFunctionIntegral(double t) {
		double term1 = c * t;
		double term2 = (a * Math.pow(t,3)) / 3;
		double term3 = (b * Math.pow(t,2)) / 2;
		return term1 + term2 + term3;
	}

	/**
	 * evaluate the double integral from 0 to t
	 * @param t time period we are double integrating over
	 * @return the double integral sum from 0 to t
	 */
	public double evaluateFunctionDoubleIntegral(double t) {
		return t * evaluateFunctionIntegral(t);
	}


	public double rangedDoubleIntegral(double t0, double t1) {
		double t1sq = Math.pow(t1,2);
		double t0sq = Math.pow(t0,2);
		double term1 = c * t1sq;
		double term2 = t1sq * t1 * b;
		double term3 = -2 * c * t1 * t0;
		double term4 = -2 * t1 * t0 * t1 * b;
		double term5Numerator1 = t1 * t1sq * (t1sq - t0sq); // this can probably be simplified a bit
		double term5Numerator2 = -t0 * t1sq * (t1sq - t0sq);
		double term5Numerator = term5Numerator1 + term5Numerator2;
		double term5 = term5Numerator / 2;
		double term6 = c * t0sq;
		double term7 = t0sq * t1 * b;
		return term1 + term2 + term3 + term4 + term5 + term6 + term7;
	}


	public void setA(double a) {
		this.a = a;
	}

	public void setB(double b) {
		this.b = b;
	}

	public void setC(double c) {
		this.c = c;
	}

	public double getA() {
		return a;
	}

	public double getB() {
		return b;
	}

	public double getC() {
		return c;
	}

	@Override
	public String toString() {
		return "" + a + "t^2 + " + b + "t +" + c;
	}
}
