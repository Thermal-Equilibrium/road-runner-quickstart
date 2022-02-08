package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.DeployIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOffIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOnIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.MutlipleAction;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;


@Autonomous
public class RedCycleRR extends BaseAuto {
	public static Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));
	Pose2d depositPosition = new Pose2d(+ 2,-TILE * 2 + 4 ,Math.toRadians(-60));
	double depositTangent = Math.toRadians(120);

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
		actions.add(new MutlipleAction(
				new action[] {
						new FollowTrajectory(robot, goToDeposit1),
						new GoToHighDeposit(robot)
				}
		));
		actions.add(new DeployIntake(robot));
		actions.add(new DepositFreight(robot));

		for (int i = 0; i < 2; i++) {

			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, goToIntake),
					new GoToInState(robot)
			}));
			actions.add(new MutlipleAction(new action[] {
					new FollowTrajectory(robot, goToIntake2),
					new TurnOnIntake(robot, true )

			}));

			actions.add(new MutlipleAction(new action[]{
					new FollowTrajectory(robot, exitWareHouse),
					new TurnOnIntake(robot, false )
			}));


			actions.add(new TurnOffIntake(robot));

			actions.add(new MutlipleAction(new action[] {
							new FollowTrajectory(robot, goToDepositCycle),
							new GoToHighDeposit(robot)
					}
			));
			actions.add(new DepositFreight(robot));
		}
		actions.add(new MutlipleAction(new action[]{
				new FollowTrajectory(robot, goToIntake),
				new GoToInState(robot)
		}));
		actions.add(new MutlipleAction(new action[] {
				new FollowTrajectory(robot, goToIntake2),
				new TurnOnIntake(robot, true )
		}));



	}
}
