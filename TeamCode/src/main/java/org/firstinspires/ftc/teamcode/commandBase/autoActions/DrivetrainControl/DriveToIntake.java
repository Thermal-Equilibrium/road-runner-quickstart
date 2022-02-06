package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

public class DriveToIntake extends DriveToPosition {
	public DriveToIntake(Robot robot, Vector3D referencePose, double cutOffTime, boolean useMaxAccel) {
		super(robot, referencePose, cutOffTime, useMaxAccel);
	}

	@Override
	public void startAction() {
		controller.resetTimer();
		robot.bucketSys.setCheckSensor(true);
		robot.bucketSys.setState(Deposit.depositStates.COLLECTION);
		robot.Deposit.setState(Deposit.depositStates.COLLECTION);
		robot.Intake.setState(Intake.intakeStates.ON);
	}

	@Override
	public boolean isActionPersistent() {
		return true;
	}

	@Override
	public boolean isActionComplete() {
		return (controller.getTime() > cutOffTime || robot.bucketSys.isFreightInBox()) && controller.getTime() > 0.5;
	}
	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
		robot.Intake.setState(Intake.intakeStates.OFF);
		robot.Deposit.setState(Deposit.depositStates.IN);
		robot.bucketSys.setState(Deposit.depositStates.IN);
		robot.bucketSys.setCheckSensor(false);
	}

}
