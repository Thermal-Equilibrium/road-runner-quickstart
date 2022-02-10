package org.firstinspires.ftc.teamcode.Controls.Coefficients;

public class PIDFCoefficients {

    public double Kp;
    public double Ki;
    public double Kd;
    public double Kd2;
    public double Kf;
    public double H;
    public double H2;

    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kd2, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Kd2 = Kd2;
        this.H = 0;
        this.H2 = 0;
    }

    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kd2, double Kf, double H) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Kd2 = Kd2;
        this.H = H;
        this.H2 = 0;
    }

    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kd2, double Kf, double H, double H2) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Kd2 = Kd2;
        this.H = H;
        this.H2 = 0;
    }
    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kd2 = 0;
        this.Kf = Kf;
        this.H = 0;
        this.H2 = 0;
    }

    public PIDFCoefficients(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kd2 = 0;
        this.Kf = 0;
        this.H = 0;
        this.H2 = 0;
    }


    /**
     * create a duplicate object without the feedforward terms
     * @return feedback only coefficients
     */
    public PIDFCoefficients noFeedforward() {
        return new PIDFCoefficients(this.Kp,this.Ki,this.Kd,0,0,0,0);
    }


}
