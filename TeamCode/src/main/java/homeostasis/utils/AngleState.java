package homeostasis.utils;

import static homeostasis.utils.AngleTools.angleWrap;

/**
 * some states are in the form of a radians angle, whi
 */
public class AngleState extends State {

    /**
     * assuming this state is desired, the oth
     *
     * @param other is the state we are using to calculate error from
     * @return the error between the two states
     */
    public State stateError(State other) {
        return new State(angleWrap(this.position - other.position), this.velocity - other.velocity);
    }

}
