package homeostasis.systems.systemIdentification;

import com.qualcomm.robotcore.util.ElapsedTime;

import homeostasis.systems.Plant;

import static homeostasis.systems.systemIdentification.findConstraints.measurementStates.END;
import static homeostasis.systems.systemIdentification.findConstraints.measurementStates.MINIMUM_POWER_TESTING;
import static homeostasis.systems.systemIdentification.findConstraints.measurementStates.START;
import static homeostasis.systems.systemIdentification.findConstraints.measurementStates.STOP;

/**
 *
 */
public class findConstraints {


    // the amount that we increase the motor power by on each iteration inorder to find the minimum power
    private static final double iterationSize = 0.01;
    // state machine variable for nonblocking code
    measurementStates STATE = START;
    // timer used for nonblocking delays and step response generation
    ElapsedTime timer = new ElapsedTime();
    // for minimum power detection we wait this many milliseconds before iterating
    private long iterationDelayMS = 10;
    // the plant we are characterizing
    private Plant plant;
    // the speed we need to reach before the minimum power has been obtained
    private double minimumSpeed;
    // current found minimum power
    private double currentMinimum = 0;

    /**
     * construct the findConstraints routine with a minimum speed and a plant
     *
     * @param plant        the system we are characterizing
     * @param minimumSpeed the minimum speed we need to reach before we conclude minimum power characterization
     */
    public findConstraints(Plant plant, double minimumSpeed) {

        this.plant = plant;
        this.minimumSpeed = minimumSpeed;

    }

    /**
     * given an online plant, use it to calculate its motion constraints
     * <p>
     * motion constraints are printed to logcat
     * <p>
     * must be called iteratively
     */
    public void measurePlantConstraints() {

        switch (STATE) {
            // start condition
            case START:
                STATE = MINIMUM_POWER_TESTING;
                timer.reset();
                break;
            // test for minimum power constraint
            case MINIMUM_POWER_TESTING:
                plant.input(currentMinimum);
                if (timer.milliseconds() > iterationDelayMS) {
                    currentMinimum += iterationSize;
                    timer.reset();
                }
                if (plant.getState().getVelocity() >= minimumSpeed) {
                    STATE = STOP;
                }
                break;
            // find maximum acceleration and velocity
            case ACCEL_VELO_TESTING:
                    STATE = END;
                break;
            // END case
            case END:

                break;
        }


    }

    public enum measurementStates {
        START,
        MINIMUM_POWER_TESTING,
        STOP,
        ACCEL_VELO_TESTING,
        END
    }

}


