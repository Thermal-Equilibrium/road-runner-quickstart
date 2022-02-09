package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
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
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;


@Autonomous
public class RedCycleRR extends BaseAuto {
	public static Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));
	Pose2d depositPosition = new Pose2d(+ 2,-TILE * 2 + 4 ,Math.toRadians(-60));
	double depositTangent = Math.toRadians(120);

	Pose2d depositPositionMid = new Pose2d(+ 2,-TILE * 2 + 4 ,Math.toRadians(-60));
	double depositTangentMid = Math.toRadians(120);

	Pose2d depositPositionLow = new Pose2d(-10,-TILE * 2 + 6.5 ,Math.toRadians(-80));
	double depositTangentLow = Math.toRadians(120);

	Pose2d intakePosition1 = new Pose2d(10, -TILE * 3 + 7.375,0);
	double intakePosition1Tangent = Math.toRadians(330);

	Pose2d intakePosition2 = new Pose2d(48, -TILE * 3 + 7.375,0);
	double intakePosition2Tangent = Math.toRadians(0);

	Pose2d intakePosition3 = new Pose2d(intakePosition2.getX() + 5, intakePosition2.getY(), intakePosition2.getHeading());
	double intakePosition3Tangent = Math.toRadians(0);

	Pose2d exitWarehouse = new Pose2d(intakePosition1.getX(), intakePosition2.getY());
	double exitWareHouseTangent = Math.toRadians(180);
	double cycleEndTangent = Math.toRadians(330 - 180);

	Pose2d intakePosition4 = new Pose2d(intakePosition2.getX() - 5, intakePosition2.getY(), Math.toRadians(0));
	double intakePosition4Tangent = Math.toRadians(0);

	Trajectory goToDeposit1;
	Trajectory goToDepositHigh;
	Trajectory goToDepositMid;
	Trajectory goToDepositLow;

	Trajectory goToIntake;
	Trajectory goToIntake2;

	Trajectory exitWareHouse;
	Trajectory goToDepositCycle;


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

		goToIntake2 = roadrunnerDrive.trajectoryBuilder(goToIntake.end(),false)
				.splineToSplineHeading(intakePosition2, intakePosition2Tangent).build();


		exitWareHouse = roadrunnerDrive.trajectoryBuilder(goToIntake2.end(), true)
				.splineToLinearHeading(exitWarehouse, exitWareHouseTangent)
				.build();

		goToDepositCycle = roadrunnerDrive.trajectoryBuilder(exitWareHouse.end(), true)
				.splineToLinearHeading(depositPosition,cycleEndTangent)
				.build();
	}

	@Override
	public void setVisionSettings() {

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

			actions.add(new DeployIntake(robot));

			//agaisnt wall slides in
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, goToIntake),
					new GoToInState(robot)
			}));
			//in warehouse intake on
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToIntake2),
					new TurnOnIntake(robot, true ),
					new Delay(2500)
			}));
			//leaves warehouse outakes
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, exitWareHouse),
					new TurnOnIntake(robot, false )
			}));
			actions.add(new TurnOffIntake(robot));
			//goes high position and deposit position
			actions.add(new MutlipleAction(new action[] {
							new FollowTrajectory(robot, goToDepositCycle),
							new GoToHighDeposit(robot)
			}));
			actions.add(new DepositFreight(robot));
			actions.add(new Delay(350));
			//agaisnt wall slides in
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, goToIntake),
					new GoToInState(robot)
			}));
			//in warehouse intake on
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToIntake2),
					new TurnOnIntake(robot, true ),
					new Delay(2500)
			}));
			//leaves warehouse outakes
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, exitWareHouse),
					new TurnOnIntake(robot, false )
			}));
			actions.add(new TurnOffIntake(robot));
			//goes high position and deposit position
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToDepositCycle),
					new GoToHighDeposit(robot)
			}));
			actions.add(new DepositFreight(robot));
			actions.add(new Delay(350));

			//agaisnt wall
			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, goToIntake),
					new GoToInState(robot)
			}));
			//parks
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToIntake2),
					new TurnOnIntake(robot, true )
			}));
			//Turns off intake
			actions.add(new Delay(500));
			actions.add(new TurnOffIntake(robot));

	}
}
