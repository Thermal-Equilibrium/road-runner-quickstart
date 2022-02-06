package homeostasis.utils;

/**
 * data type to conveniently store coupled position and velocity data
 */
public class State {

    /**
     * position state of a system
     */
    protected double position = 0;

    /**
     * velocity state of a system
     */
    protected double velocity = 0;

    /**
     * construct the state with the default 0s
     */
    public State() {

    }

    public State(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    /**
     * get the velocity state
     *
     * @return state velocity
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * set the velocity of the state
     *
     * @param velocity velocity
     */
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    /**
     * get the state position
     *
     * @return state position
     */
    public double getPosition() {
        return position;
    }

    /**
     * set the position of the state
     *
     * @param position state position
     */
    public void setPosition(double position) {
        this.position = position;
    }

    /**
     * assuming this state is desired, the other is the current state
     *
     * @param other is the state we are using to calculate error from
     * @return the error between the two states
     */
    public State stateError(State other) {
        return new State(this.position - other.position, this.velocity - other.velocity);
    }

    /**
     * add two states together
     *
     * @param other the other state
     * @return the sum of the two states
     */
    public State add(State other) {
        return new State(this.position + other.position, this.velocity + other.velocity);
    }

    /**
     * given a single gain, scale the state
     *
     * @param gain amount we are scaling the state by
     * @return scaled state vector
     */
    public State scale(double gain) {
        return new State(this.position * gain, this.velocity * gain);
    }

    /**
     * given a state containing a set of gains, scale the current states by this other gain vector
     *
     * @param gains the gains to scale the state by
     * @return scaled result
     */
    public State scale(State gains) {
        return new State(this.position * gains.position, this.velocity * gains.velocity);
    }

    /**
     * add the position and velocity states together for usage as the ability to do coupled systems
     *
     * @return the coupled output
     */
    public double coupleStates() {
        return position + velocity;
    }

    /**
     * return the negative of the current state
     *
     * @return negative state
     */
    public State negative() {
        return new State(-position, -velocity);
    }

    @Override
    public String toString() {
        return "state{" + "position=" + position + ", velocity=" + velocity + '}';
    }
}