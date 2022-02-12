package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.TapeTurret;

import java.util.ArrayList;

public class Robot {

    private final ArrayList<subsystem> subsystems = new ArrayList<>();


    public final static boolean isCompBot = true;

    public VoltageSensor batterVoltageSensor;

    public Intake Intake = new Intake();

    public Deposit Deposit = new Deposit();

    public Roadrunner driveTrain = new Roadrunner(batterVoltageSensor);

    public Bucket bucketSys = new Bucket();

    public Dashboard dashBoard = new Dashboard();

    public DuckDetection duckDetection = new DuckDetection(dashBoard);

    public DuckWheel duckwheel = new DuckWheel();

    public DeadwheelRetract retract = new DeadwheelRetract();


    public DistanceSensorLocalization distanceSensorLocalization
            = new DistanceSensorLocalization(driveTrain);

    public Robot() {



    }


    /**
     * initialize only drive train and dashboard subsystems
     *
     * @param hwmap HardwareMap instance
     */
    public void initMinimal(HardwareMap hwmap) {
        batterVoltageSensor = hwmap.voltageSensor.iterator().next();
        driveTrain.init(hwmap);
        dashBoard.init(hwmap);
        subsystems.add(driveTrain);
        subsystems.add(dashBoard);
    }

    /**
     * initialization including reset of subsystems
     *
     * @param hwmap HardwareMap instance
     */
    public void init(HardwareMap hwmap) {

        initMinimal(hwmap);
        distanceSensorLocalization.init(hwmap);
        Intake.init(hwmap);
        Deposit.init(hwmap);
        bucketSys.init(hwmap);
        duckDetection.init(hwmap);
        duckwheel.init(hwmap);
        retract.init(hwmap);
        subsystems.add(bucketSys);
        subsystems.add(Intake);
        subsystems.add(Deposit);
        subsystems.add(duckDetection);
        subsystems.add(duckwheel);
        subsystems.add(distanceSensorLocalization);
        subsystems.add(retract);

    }

    /**
     * initialization but without resetting certain subsystems
     * such as the encoder of linear slides and things that need to retain position between auto and teleop
     * @param hwmap HardwareMap instance
     */
    public void initWithoutReset(HardwareMap hwmap) {
        initMinimal(hwmap);
        distanceSensorLocalization.init(hwmap);
        Intake.init(hwmap);
        Deposit.initNoReset(hwmap);
        bucketSys.init(hwmap);
        duckDetection.init(hwmap);
        duckwheel.init(hwmap);
        retract.init(hwmap);
        subsystems.add(bucketSys);
        subsystems.add(Intake);
        subsystems.add(Deposit);
        subsystems.add(duckDetection);
        subsystems.add(duckwheel);
        subsystems.add(distanceSensorLocalization);
        subsystems.add(retract);
    }

    /**
     * obtain the robot position
     *
     * @return drivetrain position
     */
    public Vector3D getRobotPose() {
        return driveTrain.subsystemState();
    }
    public void setRobotPose(Vector3D pose) {
        driveTrain.setPositionEstimate(pose);
    }

    /**
     * @return array list of subsystems
     */
    public ArrayList<subsystem> getSubsystems() {
        return subsystems;
    }


    public Vector3D getVelocity() {
        return new Vector3D(0,0,0);
    }



}
