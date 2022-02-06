package homeostasis.controllers;

import homeostasis.systems.Plant;
import homeostasis.utils.State;

/**
 * state feedback inorder to regulate a systems state to 0
 * <p>
 * This is an example of a MISO feedback regulator
 * <p>
 * We call this a regulator as it drives the states to 0, we can use subtraction to turn this into a reference
 * tracking controller
 * <p>
 * the feedback diagram for this regulator can be drawn as
 */
public class StateRegulator {

    /**
     * the system we are regulating
     */
    protected Plant plant;
    protected State gains;
    protected State controllerOffset = new State(0, 0);

    /**
     * set up the state regulator with the system we would like to regulate
     *
     * @param plant the system we would like to regulate to 0 on each of its states
     */
    public StateRegulator(Plant plant, State regulatorGain) {
        this.plant = plant;
        this.gains = regulatorGain;
    }

    public double controllerOutput() {
        State currentState = plant.getState();
        System.out.println("current state is " + currentState);
        return controllerOffset.stateError(currentState).scale(gains).coupleStates();
    }
    /**
     * regulate the plants state to 0
     * <p>
     * full state feedback takes the form: u = -kx
     * <p>
     * where:
     * u is the input to our system
     * x is the current state of our system
     * k is our controller gain
     */
    public void regulatePlant() {
        State currentState = plant.getState();
        System.out.println("current state is " + currentState);
        plant.input(controllerOutput());
    }


}
