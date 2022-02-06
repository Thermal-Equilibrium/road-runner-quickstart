package org.firstinspires.ftc.teamcode.commandBase;

/**
 * action that is used to run in the teleop period
 */
public interface teleopAction {

	/**
	 * is executed on the first run of the action, generally when the button is first pressed
	 */
	void initialRun();

	/**
	 * periodically ran until the action is complete
	 */
	void periodic();

	/**
	 * signal that the action is complete
	 *
	 * @return true if complete
	 */
	boolean isComplete();

	/**
	 * assess if we should run this action
	 *
	 * @return true if we should run this action
	 */
	boolean shouldRun();

	/**
	 * reset states to where the action can be used again
	 */
	void reset();

	/**
	 * check if the initial run has occurred
	 *
	 * @return returns true if this is the case
	 */
	boolean hasPerformedInitialRun();
}
