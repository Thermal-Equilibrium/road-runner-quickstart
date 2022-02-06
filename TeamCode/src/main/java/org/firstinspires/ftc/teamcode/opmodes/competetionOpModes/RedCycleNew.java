package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.LastResort.TimeBasedMove;
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
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.SetYPosition;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Autonomous
public class RedCycleNew extends BaseAuto {

	Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));

	Vector3D depositPosition1HIGH = new Vector3D(+ 2,-TILE * 2 + 4 ,Math.toRadians(-60));
	Vector3D depositPosition1MID  = new Vector3D(+ 2,-TILE * 2 + 2 ,Math.toRadians(-60));
	Vector3D depositPosition1LOW  = new Vector3D(+ -3,-TILE * 2 + 8 ,Math.toRadians(-60));

	Vector3D depositPosition1 = new Vector3D(+ .5,-TILE * 2 + 8 ,Math.toRadians(-50));
	Vector3D depositPosition2 = new Vector3D(+ .5,-TILE * 2 + 8 ,Math.toRadians(-60));

	Vector3D readyForPark = new Vector3D(TILE / 3, -TILE * 3 + 11, Math.toRadians(0));

	Vector3D LineUp1 = new Vector3D(TILE - 18, -TILE * 3 + 18 ,  depositPosition1.getAngleRadians());
	Vector3D LineUp2 = new Vector3D(TILE - 18, TILE * -3 + 6,  Math.toRadians(0));

	Vector3D StrafeIntoWall = new Vector3D(0,.5,0);

	Vector3D newcollect1 = new Vector3D(TILE * 3 - 26 ,-TILE * 3 + 5.5, Math.toRadians(0));
	Vector3D newcollect2 = new Vector3D(TILE * 3 - 22,-TILE * 3 + 5.5, Math.toRadians(0));
	Vector3D newcollect3 = new Vector3D(TILE * 3 - 20 ,-TILE * 3 + 5.5, Math.toRadians(0));

	Vector3D NewReadyForDepo1 = new Vector3D(TILE - 15, -TILE * 3 + 5.,  Math.toRadians(0));
	Vector3D NewReadyForDepo2 = new Vector3D(TILE - 15, -TILE * 3 + 5.25,  Math.toRadians(0));
	Vector3D NewReadyForDepo3 = new Vector3D(TILE - 15, -TILE * 3 + 5.5,  Math.toRadians(0));

	protected double turn_cutoff_time_seconds = 0.7;

	@Override
	public void setStartingPosition() {
		robot.setRobotPose(start);
	}

	@Override
	public void setVisionSettings() {

	}

	public void depositPreload() {
		Vector3D targetPosition = depositPosition1;

		switch (TSEPosition) {
			case LEFT:
				actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1LOW), new DeployIntake(robot), new NoSlideDeposit(robot)}));
				//actions.add();
				actions.add(new DepositFreight(robot));
				actions.add(new Delay(1300));
				break;
			case MIDDLE:
				actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1MID),
						new GoToMidDeposit(robot), new DeployIntake(robot)}));
				actions.add(new DepositFreight(robot));
				actions.add(new Delay(300));
				break;
			case RIGHT:
				actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1HIGH),
						new GoToHighDeposit(robot), new DeployIntake(robot)}));
				actions.add(new DepositFreight(robot));
				actions.add(new Delay(300));
				break;
		}
	}


	@Override
	public void addActions() {

		//Deposit pre-load Works
		depositPreload();

		//agaisnt wall Meh
		actions.add(new DriveToPosition(robot, LineUp1,1.5,false));
		actions.add(new DriveToPosition(robot,LineUp2,1.5,false));
		actions.add(new MutlipleAction(new action[]{new TimeBasedMove(robot,StrafeIntoWall,1), new GoToInState(robot)}));

		actions.add(new SetYPosition(robot,-TILE * 3 + 6 ));

		//Intake Meh goes to far
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,newcollect1), new TurnOnIntake(robot,true),new Delay(500)}));
		actions.add(new TurnOffIntake(robot));
		//actions.add(new DriveToIntake(robot,newcollect1,3,false))

		//out of depo
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, NewReadyForDepo1,2,false), new TurnOnIntake(robot,false)}));
		actions.add(new TurnOffIntake(robot));

//------------------------------------------------------------------------------------------------\\
		//Deposit pre-load
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition2), new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));



		/*
		//agaisnt wall WITH TIME BASED CODE
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, LineUp1,2,false), new GoToInState(robot)}));
		actions.add(new TimeBasedMove(robot,StrafeIntoWall,.75));
		actions.add(new SetYPosition(robot,-TILE * 3 + 6 ));
		//Intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,newcollect2), new TurnOnIntake(robot,true),new Delay(500)}));
		actions.add(new TurnOffIntake(robot));
		//out of depo
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, NewReadyForDepo2,2,false), new TurnOnIntake(robot,false)}));
		actions.add(new TurnOffIntake(robot));
//------------------------------------------------------------------------------------------------\\
		//Deposit pre-load
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition2), new GoToHighDeposit(robot), new DeployIntake(robot), new Delay(250)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, LineUp1), new GoToInState(robot)}));
		actions.add(new SetYPosition(robot,-TILE * 3 + 6 ));

		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, newcollect3,2,false), new TurnOnIntake(robot,true)}));
		actions.add(new TurnOffIntake(robot));

/*
		//deposit first preload
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1), new GoToHighDeposit(robot), new DeployIntake(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));
		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1) , new GoToInState(robot)}));

		//Intake first freight
		actions.add(new Turn(robot,0,turn_cutoff_time_seconds));
		actions.add(new DriveToIntake(robot, collect1, 3.5, false));

		//Exit warehouse
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1,2,false) , new TurnOnIntake(robot,false), new Delay(250)}));
		actions.add(new TurnOffIntake(robot));
		//actions.add(new DriveToPosition(robot, readyForCollection1,1.5,false));

//------------------------------------------------------------------------------------------------\\

		//Deposit 1st cubeadb connect 192.168.43.1:5555
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection2) , new GoToInState(robot)}));

		//Intake first freight
		//actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,collect2,2.5,false) , new TurnOnIntake(robot,true)}));
		//actions.add(new TurnOffIntake(robot));
		actions.add(new Turn(robot,0, turn_cutoff_time_seconds));
		actions.add(new DriveToIntake(robot, collect2, 3.5, false));

		//Exit warehouse
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection2,2,false) , new TurnOnIntake(robot,false), new Delay(250)}));
		actions.add(new TurnOffIntake(robot));

//------------------------------------------------------------------------------------------------\\

		//Deposit 2nd cube
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection3) , new GoToInState(robot)}));

		//Intake first freight
		//actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,collect3,2.5,false) , new TurnOnIntake(robot,true)}));
		//actions.add(new TurnOffIntake(robot));
		actions.add(new Turn(robot,0, turn_cutoff_time_seconds));
		actions.add(new DriveToIntake(robot, collect3, 3.5, false));

		//Exit warehouse
*/
	}


}
