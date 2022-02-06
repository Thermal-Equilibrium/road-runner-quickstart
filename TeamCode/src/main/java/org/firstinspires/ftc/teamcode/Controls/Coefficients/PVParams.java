package org.firstinspires.ftc.teamcode.Controls.Coefficients;

public class PVParams {

	private double Kp; // position proportional
	private double KpV; // velocity proportional
	private double Kv; // velocity feedforward
	private double Ks; // static feedforward
	private double Aff; // adaptive feedforward
	private double cutOffVelo; // minimum veloctiy before exit
	private double cutOffPos; // minimum position before exit
	private double Ki; // integral gain
	public PVParams(double Kp, double KpV, double Ki, double Kv, double Ks, double Aff, double cutoffvelo, double cutoffpos) {
		this.Kp = Kp;
		this.KpV = KpV;
		this.Kv = Kv;
		this.Ks = Ks;
		this.Aff = Aff;
		this.Ki = Ki;
		this.cutOffVelo = cutoffvelo;
		this.cutOffPos = cutoffpos;
	}


	public double getKp() {
		return Kp;
	}

	public void setKp(double kp) {
		Kp = kp;
	}

	public double getKpV() {
		return KpV;
	}

	public void setKpV(double kpV) {
		KpV = kpV;
	}

	public double getKv() {
		return Kv;
	}

	public void setKv(double kv) {
		Kv = kv;
	}

	public double getKs() {
		return Ks;
	}

	public void setKs(double ks) {
		Ks = ks;
	}

	public double getAff() {
		return Aff;
	}

	public void setAff(double aff) {
		Aff = aff;
	}

	public double getCutOffVelo() {
		return cutOffVelo;
	}

	public double getCutOffPos() {
		return cutOffPos;
	}

	public void setCutOffPos(double cutOffPos) {
		this.cutOffPos = cutOffPos;
	}


	public double getKi() {
		return Ki;
	}
}
