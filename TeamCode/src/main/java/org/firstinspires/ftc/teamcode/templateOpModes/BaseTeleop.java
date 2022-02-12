package org.firstinspires.ftc.teamcode.templateOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.commandBase.Scheduler;
import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.opmodes.FieldSide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.alliance;

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
//		robot.odometry.setState(ThreeWheelOdometry.OdomState.DEPLOYED);

		switch (alliance) {
			case RED:
				robot.setRobotPose(new Vector3D(0,0,Math.toRadians(0)));
				break;
			case BLUE:
				robot.setRobotPose(new Vector3D(0,0,Math.toRadians(180)));
				break;
		}

		robot.retract.retract();


		addActions();
		scheduler = new Scheduler(robot.getSubsystems(), actions, hardwareMap);

		waitForStart();
		while (opModeIsActive()) {
			scheduler.updateTeleop();
		}

	}
}
