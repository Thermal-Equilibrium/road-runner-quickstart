package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients;
import org.firstinspires.ftc.teamcode.Controls.SISOControls.RobustPID;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import java.util.ArrayList;

import homeostasis.systems.DcMotorPlant;
import homeostasis.utils.State;

public class Slides implements subsystem {

	VoltageSensor batterVoltageSensor;

	protected RobustPID slideController = new RobustPID(controllerCoefficients.slideCoefficients, 0, 5, 3, 2);
	protected DcMotorPlant slides;
	protected DcMotorEx left;
	protected DcMotorEx right;

	protected Deposit.depositStates state = Deposit.depositStates.IN;

	protected double IN = 0;
	protected double COLLECTION = 0;
	protected double LOW = 60;
	protected double MID = 140;
	protected double HIGH = 260; // tune this imo

	protected double referencePosition = 0;

	protected double error = 100;

	@Override
	public void init(HardwareMap hwmap) {
		initNoReset(hwmap);
		slides.resetEncoder();
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		left = hwmap.get(DcMotorEx.class, "slideLeft");
		right = hwmap.get(DcMotorEx.class, "slideRight");
		batterVoltageSensor = hwmap.voltageSensor.iterator().next();

		right.setDirection(DcMotorSimple.Direction.FORWARD);
		left.setDirection(DcMotorSimple.Direction.REVERSE);
		left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		ArrayList<DcMotorEx> motors = new ArrayList<>();
		motors.add(right);
		motors.add(left);
		slides = new DcMotorPlant(motors);
		slideController.setLimitIntegralSum(false);
		slideController.setH2Cutoff(5);
	}

	/**
	 * periodically called, updates state machine
	 */
	@Override
	public void update() {
		// if we are disarmed, we need not send power to our slides.
		if (state.equals(Deposit.depositStates.DISARMED)) {
			slides.input(0);
			return;
		}

		switch (state) {
			case IN:
				referencePosition = IN;
				break;
			case COLLECTION:
				referencePosition = COLLECTION;
				break;
			case GOING_TO_LOW_2:
			case GOING_TO_MID:
			case AT_MID:
				referencePosition = MID;
				break;
			case GOING_TO_HIGH:
			case AT_HIGH:
				referencePosition = HIGH;
				break;
			case GOING_TO_LOW:
			case AT_LOW:
				referencePosition = LOW;
				break;
			case DEPOSITING:
			case GOING_IN:
			case AT_LOWEST:
				break;

		}
		System.out.println("slide reference position is " + referencePosition);
		slideController.setScaling(14 / batterVoltageSensor.getVoltage());
		double controllerCommand = slideController.stateReferenceCalculate(referencePosition, subsystemState().getPosition());
		slides.input(controllerCommand);
		error = slideController.getError();
		System.out.println("slide controller error is " + error + " slide controller setpoint is " + slideController.getReference());
		System.out.println("slide power " + controllerCommand);
		Dashboard.packet.put("Current",left.getCurrent(CurrentUnit.AMPS) + right.getCurrent(CurrentUnit.AMPS));
		Dashboard.packet.put("SlidePower",controllerCommand);
		Dashboard.packet.put("slide error", error);
	}

	public double getControllerError() {
		return error;
	}

	/**
	 * sets the state of our system
	 *
	 * @param state state as defined in the enum in the deposit class
	 */
	public void setState(Deposit.depositStates state) {
		this.state = state;
	}

	/**
	 * gets the position and velocity of the slide system
	 *
	 * @return position and velocity as state object
	 */
	@Override
	public State subsystemState() {
		return slides.getState();
	}

}
