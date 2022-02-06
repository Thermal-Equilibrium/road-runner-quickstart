package org.firstinspires.ftc.teamcode.Filter;

public class LowPassFilter {

    /**
     * the low pass filter gain (0 < a < 1)
     */
    public double a;
    /**
     * the previous estimate of the low pass filter
     */
    public double previousFilterEstimate = 0;

    /**
     * construct the low pass filter object
     *
     * @param a low pass filter gain (0 < a < 1)
     */
    public LowPassFilter(double a) {

        if (a > 1) {
            a = 1;
        }
        if (a < 0) {
            a = 0;
        }

        this.a = a;
    }

    public double updateEstimate(double measurement) {
        double currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * measurement;
        previousFilterEstimate = currentFilterEstimate;
        return currentFilterEstimate;
    }

}
