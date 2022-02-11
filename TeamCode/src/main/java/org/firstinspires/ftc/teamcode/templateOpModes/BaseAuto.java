package org.firstinspires.ftc.teamcode.templateOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.Scheduler;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.TSEContourPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;

	public final static double TILE = 24;

	protected ArrayList<action> actions = new ArrayList<>();
	protected Vector3D startingPosition = new Vector3D();

	protected TSEContourPipeline.position TSEPosition = TSEContourPipeline.position.RIGHT;

	public abstract void setStartingPosition();

	public abstract void setVisionSettings();

	public abstract void addActions();

	public SampleMecanumDrive roadrunnerDrive;

	@Override
	public void runOpMode() {
		robot = new Robot();
		if (isCompBot) {
			robot.init(hardwareMap);
			setVisionSettings();
		} else {
			robot.initMinimal(hardwareMap);
		}

		this.roadrunnerDrive = robot.driveTrain.mecanumDrive;

		this.robot.bucketSys.setCheckSensor(false);
		robot.retract.down();
		setStartingPosition();



		while (!isStopRequested() && !isStarted()) {
			TSEPosition = robot.duckDetection.subsystemState();
			telemetry.addData("Current TSE Position is: ", TSEPosition);
			telemetry.update();
		}


		waitForStart();
		System.out.println("Final detected position was " + TSEPosition);

		addActions();
		Scheduler scheduler = new Scheduler(hardwareMap, actions, robot.getSubsystems());

		while (opModeIsActive()) {
			scheduler.updateStateMachineAndRobot();
		}

	}

	public void setVisionForRightVisible() {
		robot.duckDetection.set_side(true);
	}

	public void setVisionForLeftVisible() {
		robot.duckDetection.set_side(false);
	}
}
