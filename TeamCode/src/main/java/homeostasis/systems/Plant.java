package homeostasis.systems;

import homeostasis.utils.Constraints;
import homeostasis.utils.State;

/**
 * in control theory, the plant is the system we would like to control
 */
public class Plant {


    protected Constraints constraints = new Constraints(0, 0, 0);

    /**
     * systems state
     *
     * @return the system state
     */
    public State getState() {
        return null;
    }

    /**
     * input to the plant
     *
     * @param input to the plant
     */
    public void input(double input) {

    }

    /**
     * obtain the plants constraints on velocity, acceleration, and minimum motor power
     *
     * @return constraints object
     */
    public Constraints getConstraints() {
        return constraints;
    }
}
