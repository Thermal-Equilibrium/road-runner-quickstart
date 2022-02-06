package homeostasis.utils;

public class Constraints extends State {

    protected double acceleration;

    // minimum power command for the robot to begin moving
    protected double minimumPower = 0;

    /**
     * construct the constraints object
     *
     * @param velocity     maximum velocity of the system
     * @param acceleration maximum acceleration of the system
     */
    public Constraints(double velocity, double acceleration) {
        this.position = 0;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    /**
     * construct the constraints object
     *
     * @param velocity     maximum velocity of the system
     * @param acceleration maximum acceleration of the system
     * @param minimumPower the minimum motor power
     */
    public Constraints(double velocity, double acceleration, double minimumPower) {
        this.position = 0;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.minimumPower = minimumPower;
    }

    /**
     * get the acceleration constraint
     *
     * @return get acceleration constraint
     */
    public double getAcceleration() {
        return acceleration;
    }

    /**
     * set the acceleration constraint
     *
     * @param acceleration acceleration constraint
     */
    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    /**
     * calculate basic velocity feedforward gain
     *
     * @return basic feedforward gain
     */
    public double getBasicFeedforward() {
        return 1 / this.velocity;
    }

    /**
     * calculate a basic nonlinear feedforward output to account for static friction at low motor power
     *
     * @param targetVelocity the target velocity we would like the motor to travel at
     * @return motor output
     */
    public double feedforwardOutput(double targetVelocity) {
        if (targetVelocity > 0) {
            return Math.max(targetVelocity * getBasicFeedforward(), minimumPower);
        } else if (targetVelocity < 0) {
            return Math.min(targetVelocity * getBasicFeedforward(), minimumPower);
        }
        return 0;
    }

    @Override
    public String toString() {
        return "constraints{" +
                " velocity=" + velocity +
                ", acceleration=" + acceleration +
                ", minimumPower=" + minimumPower +
                '}';
    }

    /**
     * set minimum power constraint
     *
     * @param minimumPower minimum power constraint
     */
    public void setMinimumPower(double minimumPower) {
        this.minimumPower = minimumPower;
    }
}
