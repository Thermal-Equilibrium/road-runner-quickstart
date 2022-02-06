package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.DeployIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Disabled
public class RedCycleAuto extends BaseAuto {

    Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));

    Vector3D depositPosition1 = new Vector3D(+ 4,-TILE * 2 + 5 ,Math.toRadians(-55));

    Vector3D readyForCollection1 = new Vector3D(TILE - 10 , -TILE * 3 + 7, Math.toRadians(0));
    Vector3D readyForCollection2 = new Vector3D(TILE -10, -TILE * 3 + 8, Math.toRadians(0));
    Vector3D readyForCollection3 = new Vector3D(TILE / 3, -TILE * 3 + 10.5, Math.toRadians(0));
    Vector3D readyForCollection4 = new Vector3D(TILE / 3, -TILE * 3 + 10.5, Math.toRadians(0));
    Vector3D readyForCollection5 = new Vector3D(TILE / 3, -TILE * 3 + 10.5, Math.toRadians(0));
    Vector3D readyForCollection6 = new Vector3D(TILE / 3, -TILE * 3 + 10.5, Math.toRadians(0));

    Vector3D readyForPark = new Vector3D(TILE / 3, -TILE * 3 + 11, Math.toRadians(0));

    Vector3D collect1 = new Vector3D(TILE * 2 - 8, readyForCollection1.getY(), Math.toRadians(0));
    Vector3D collect2 = new Vector3D(TILE * 2, readyForCollection2.getY(), Math.toRadians(0));
    Vector3D collect3 = new Vector3D(TILE * 2, readyForCollection3.getY(), Math.toRadians(0));
    Vector3D collect4 = new Vector3D(TILE * 2, readyForCollection4.getY(), Math.toRadians(0));
    Vector3D collect5 = new Vector3D(TILE * 2, readyForCollection5.getY(), Math.toRadians(0));
    Vector3D collect6 = new Vector3D(TILE * 2, readyForCollection6.getY(), Math.toRadians(0));
    Vector3D park = new Vector3D(TILE * 2 - 6, readyForCollection6.getY(), Math.toRadians(0));

    Vector3D gapPose = new Vector3D(TILE, readyForPark.getY(), Math.toRadians(0));
    Vector3D Test = new Vector3D(24,24, Math.toRadians(0));

    Vector3D InWarehouse1 = new Vector3D(TILE * 2 - 10, readyForCollection1.getY(), Math.toRadians(0));
    Vector3D InWarehouse2 = new Vector3D(TILE * 2 - 6, readyForCollection2.getY(), Math.toRadians(0));
    Vector3D InWarehouse3 = new Vector3D(TILE * 2 - 6, readyForCollection3.getY(), Math.toRadians(0));

    @Override
    public void setStartingPosition() {
        robot.odometry.setPositionEstimate(start);
    }

    @Override
    public void setVisionSettings() {

    }

    @Override
    public void addActions() {

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DeployIntake(robot));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForCollection1));

        //Intake first freight
        actions.add(new DriveToIntake(robot,collect1,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection1));

        //--------------------------------------------------------------------//

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForCollection2));

        //Intake first freight
        actions.add(new DriveToIntake(robot,collect2,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection2));

        //--------------------------------------------------------------------//

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForCollection3));

        //Intake first freight
        actions.add(new DriveToIntake(robot,collect3,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection3));

        //--------------------------------------------------------------------//

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForCollection4));

        //Intake first freight
        actions.add(new DriveToIntake(robot,collect4,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection4));

        //--------------------------------------------------------------------//

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForCollection5));

        //Intake first freight
        actions.add(new DriveToIntake(robot,collect5,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection5));

        //--------------------------------------------------------------------//

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForCollection6));

        //Intake first freight
        actions.add(new DriveToIntake(robot,collect6,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection6));

        //--------------------------------------------------------------------//

        //Deposit pre-load
        actions.add(new DriveToPosition(robot,depositPosition1));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for first warehouse cycle + deploy intake
        actions.add(new DriveToPosition(robot,readyForPark));

        actions.add(new DriveToPosition(robot,park));


        //--------------------------------------------------------------------//







































  /*
        //Deposit first freight
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for second warehouse cycle
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection2) , new GoToInState(robot)}));

        //Intake second freight
        actions.add(new DriveToIntake(robot,collect2,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection2));

        //Deposit second freight
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for third warehouse cycle
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection3) , new GoToInState(robot)}));

        //Intake third freight
        actions.add(new DriveToIntake(robot,collect3,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection3));

        //Deposit third freight
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for fourth warehouse cycle
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection4) , new GoToInState(robot)}));

        //Intake fourth freight
        actions.add(new DriveToIntake(robot,collect4,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection4));

        //Deposit fourth freight
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for fifth warehouse cycle
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection5) , new GoToInState(robot)}));

        //Intake fifth freight
        actions.add(new DriveToIntake(robot,collect5,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection5));

        //Deposit fifth freight
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for sixth warehouse cycle
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection6) , new GoToInState(robot)}));

        //Intake sixth freight
        actions.add(new DriveToIntake(robot,collect6,5,false));

        //Exit warehouse
        actions.add(new DriveToPosition(robot, readyForCollection6));

        //Deposit sixth freight
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
        actions.add(new DepositFreight(robot));
        actions.add(new Delay(250));

        //Against wall lineup for warehouse park
        actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForPark) , new GoToInState(robot)}));

        //Park in warehouse
        actions.add(new DriveToPosition(robot,park));*/
    }
}
