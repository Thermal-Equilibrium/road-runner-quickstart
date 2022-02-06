package homeostasis.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import homeostasis.Filters.VelocityKalmanFilter;
import homeostasis.utils.State;

public class DcMotorPlant extends Plant {

    protected State motorState = new State(0, 0);

    protected DcMotorEx motor;

    /**
     * the previous value sent to our motor for lynx read optimizations
     */
    protected double previousMotorInput = 0;

    /**
     * arraylist of our dcmotors, the first motor is the motor attached to our sensor.
     */
    ArrayList<DcMotorEx> motors = new ArrayList<>();

    /**
     * filter for estimating velocity
     */
    VelocityKalmanFilter velocityObserver;

    /**
     * construct and initialize motor
     *
     * @param motorName name of the motor in the hardware map
     * @param hwmap     hardware map instance
     */
    public DcMotorPlant(String motorName, HardwareMap hwmap) {

        motor = hwmap.get(DcMotorEx.class, motorName);
        motors.add(motor);
        velocityObserver = new VelocityKalmanFilter(motor);

    }

    /**
     * construct DcMotor System with an already initialized motor
     *
     * @param dcMotor DcMotorEx object
     */
    public DcMotorPlant(DcMotorEx dcMotor) {
        motor = dcMotor;
        motors.add(motor);
        velocityObserver = new VelocityKalmanFilter(motor);
    }

    /**
     * construct a coupled dc motor system
     * <p>
     * the first DcMotor in the list will be used as the observer (the encoder will read from this one)
     *
     * @param motorList list of linked motors
     */
    public DcMotorPlant(ArrayList<DcMotorEx> motorList) {
        this.motors = motorList;
        velocityObserver = new VelocityKalmanFilter(motorList.get(0));
    }


    /**
     * measure the first motor as the state
     *
     * @return system state
     */
    @Override
    public State getState() {
        this.motorState = new State(motors.get(0).getCurrentPosition(), velocityObserver.lowPassVelocity());
        return motorState;
    }

    /**
     * set input to a coupled set of motors
     *
     * @param input to the plant
     */
    @Override
    public void input(double input) {

        input = Range.clip(input, -1, 1);

        if (input != previousMotorInput) {
            for (DcMotorEx motor : motors) {
                motor.setPower(input);
            }
        }

        previousMotorInput = input;

    }

    /**
     * reset motor encoders to drip
     */
    public void resetEncoder() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }


}
