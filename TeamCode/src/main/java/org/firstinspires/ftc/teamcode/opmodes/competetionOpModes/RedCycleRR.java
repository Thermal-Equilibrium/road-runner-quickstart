package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.DeployIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOffIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOnIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.MutlipleAction;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToMidDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.NoSlideDeposit;
import org.firstinspires.ftc.teamcode.opmodes.FieldSide;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.ALLIANCE.BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.ALLIANCE.RED;


@Autonomous
public class RedCycleRR extends BaseAuto {

	double cycleDistanceFromWallY = -TILE * 3 + 5.675;
	public static Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));
	Pose2d depositPosition = new Pose2d(+ 2,-TILE * 2 + 2.5  ,Math.toRadians(-65));
	double depositTangent = Math.toRadians(120);

	Pose2d depositPositionMid = new Pose2d(+ 2,-TILE * 2 + 5 ,Math.toRadians(-65));
	double depositTangentMid = Math.toRadians(120);

	Pose2d depositPositionLow = new Pose2d(-8,-TILE * 2 + 5,Math.toRadians(-85));
	double depositTangentLow = Math.toRadians(120);

	Pose2d intakePosition1 = new Pose2d(10, cycleDistanceFromWallY,0);
	double intakePosition1Tangent = Math.toRadians(330);

	Pose2d intakePosition2A = new Pose2d(46, cycleDistanceFromWallY,0);
	double intakePosition2Tangent = Math.toRadians(0);

	Pose2d intakePosition2B = new Pose2d(48, cycleDistanceFromWallY,0);



	Pose2d exitWarehouse = new Pose2d(intakePosition1.getX(), intakePosition2A.getY());
	double exitWareHouseTangent = Math.toRadians(180);
	double cycleEndTangent = Math.toRadians(330 - 180);

	Trajectory goToDeposit1;
	Trajectory goToDepositHigh;
	Trajectory goToDepositMid;
	Trajectory goToDepositLow;

	Trajectory goToIntake;
	Trajectory goToIntake2A;
	Trajectory goToIntake2B;

	Trajectory exitWareHouse;
	Trajectory goToDepositCycle;

	Trajectory[] intake2Options;



	@Override
	public void setStartingPosition() {
		robot.setRobotPose(start);
		goToDeposit1 = roadrunnerDrive.trajectoryBuilder(start.toPose2d(), true)
				.splineToLinearHeading(depositPosition, depositTangent)
				.build();

		goToDepositHigh = roadrunnerDrive.trajectoryBuilder(start.toPose2d(), true)
				.splineToLinearHeading(depositPosition, depositTangent)
				.build();

		goToDepositMid = roadrunnerDrive.trajectoryBuilder(start.toPose2d(), true)
				.splineToLinearHeading(depositPositionMid, depositTangentMid)
				.build();

		goToDepositLow = roadrunnerDrive.trajectoryBuilder(start.toPose2d(), true)
				.splineToLinearHeading(depositPositionLow, depositTangentLow)
				.build();

		goToIntake = roadrunnerDrive.trajectoryBuilder(goToDeposit1.end(),false)
				.splineToLinearHeading(intakePosition1, intakePosition1Tangent).build();

		goToIntake2A = roadrunnerDrive.trajectoryBuilder(goToIntake.end(),false)
				.splineToSplineHeading(intakePosition2A, intakePosition2Tangent).build();

		goToIntake2B = roadrunnerDrive.trajectoryBuilder(goToIntake.end(),false)
				.splineToSplineHeading(intakePosition2B, intakePosition2Tangent).build();


		exitWareHouse = roadrunnerDrive.trajectoryBuilder(goToIntake2A.end(), true)
				.splineToLinearHeading(exitWarehouse, exitWareHouseTangent)
				.build();

		goToDepositCycle = roadrunnerDrive.trajectoryBuilder(exitWareHouse.end(), true)
				.splineToLinearHeading(depositPosition,cycleEndTangent)
				.build();


		intake2Options = new Trajectory[]{goToIntake2A, goToIntake2B};
	}

	@Override
	public void setVisionSettings() {
		FieldSide.alliance = RED;
	}

	@Override
	public void addActions() {

		// this makes sure the robot starts with the bucket all the way out instead of flipping back
		actions.add(new NoSlideDeposit(robot));
		switch (TSEPosition) {

			case LEFT:
				actions.add(new MutlipleAction(
						new action[] {
								new FollowTrajectory(robot, goToDepositLow),
						}
				));
				actions.add(new NoSlideDeposit(robot));
				break;

			case MIDDLE:
				actions.add(new MutlipleAction(
						new action[] {
								new FollowTrajectory(robot, goToDepositMid),
								new GoToMidDeposit(robot)
						}
				));
				actions.add(new DepositFreight(robot));
				break;

			case RIGHT:
				actions.add(new MutlipleAction(
						new action[] {
								new FollowTrajectory(robot, goToDepositHigh),
								new GoToHighDeposit(robot)
						}
				));
				actions.add(new DepositFreight(robot));
				break;
		}

			actions.add(new DepositFreight(robot));
			actions.add(new DeployIntake(robot));
			actions.add(new Delay(250));

		for (int i = 0; i < 2; i ++) {
			//agaisnt wall slides in
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, goToIntake),
					new GoToInState(robot)
			}));
			//in warehouse intake on
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, intake2Options[i]),
					new TurnOnIntake(robot, true ),
					new Delay(2500)
			}));

			actions.add(new MutlipleAction(new action[] {
					new TurnOnIntake(robot, false),
					new Delay(600)
			}));
			actions.add(new TurnOffIntake(robot));


			//leaves warehouse outakes
			actions.add(new FollowTrajectory(robot, exitWareHouse));
			//goes high position and deposit position
			actions.add(new GoToHighDeposit(robot));
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToDepositCycle),
			}));
			actions.add(new DepositFreight(robot));
			actions.add(new Delay(250));
		}

			//agaisnt wall
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, goToIntake),
					new GoToInState(robot)
			}));
			//parks
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToIntake2A),
			}));


	}
	//E A SPORTS
}
