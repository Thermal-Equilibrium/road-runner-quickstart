package homeostasis.controllers;

import homeostasis.systems.Plant;
import homeostasis.utils.State;

/**
 * MISO (Multiple input, single output) state space controller that allows us to control position and velocity at the
 * same time by coupling the states together in the regulator
 * <p>
 * A controller is different than a regulator as the regulator simply drives the desired state to 0.
 * <p>
 * A controller is more useful as it allows us to drive our system to any state within the state space
 * (assuming the system is controllable and observable).
 * <p>
 * Given the plot of x dot vs x, the state controller can reach (but not necessarily maintain) any point in state space.
 * <p>
 * | - x dot axis
 * |
 * |       * - potential place a system could be in state space
 * |
 * ----------------|---------------  - x axis
 * |
 * |
 * |
 * <p>
 * where as a regulator simply forces these states towards 0
 */
public class StateController extends StateRegulator {

    protected double Kv = 0;

    /**
     * set up the state regulator with the system we would like to control
     *
     * @param plant         the system of which contains the states we would like to regulate to 0.
     * @param regulatorGain the gain of our regulator.
     */
    public StateController(Plant plant, State regulatorGain) {
        super(plant, regulatorGain);
    }
    public StateController(Plant plant, State regulatorGain, double Kv) {
        super(plant, regulatorGain);
        this.Kv = Kv;
    }


    /**
     * Use the regulator to drive the state error to 0
     */

    public void controlState(State desired) {
        this.controllerOffset = desired;
        State currentState = plant.getState();
        System.out.println("current state is " + currentState);
        plant.input(controllerOutput() + (desired.getVelocity() * Kv));
    }


}
