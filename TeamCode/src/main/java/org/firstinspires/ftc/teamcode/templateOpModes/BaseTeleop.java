package org.firstinspires.ftc.teamcode.templateOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.commandBase.Scheduler;
import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ThreeWheelOdometry;

import java.util.ArrayList;

public class BaseTeleop extends LinearOpMode {


	protected Robot robot;
	protected Scheduler scheduler;
	protected ArrayList<teleopAction> actions = new ArrayList<>();

	public void addActions() {

	}

	@Override
	public void runOpMode() {
		robot = new Robot();
		robot.init(hardwareMap);
		robot.odometry.setState(ThreeWheelOdometry.OdomState.DEPLOYED);
		robot.odometry.setPositionEstimate(PoseStorage.pose);

		addActions();
		scheduler = new Scheduler(robot.getSubsystems(), actions, hardwareMap);

		waitForStart();
		while (opModeIsActive()) {
			scheduler.updateTeleop();
		}

	}
}
