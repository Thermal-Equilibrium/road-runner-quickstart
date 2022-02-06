package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.Utils.utils;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import homeostasis.systems.DcMotorPlant;

import static org.firstinspires.ftc.teamcode.Utils.utils.fromPose2D;
import static org.firstinspires.ftc.teamcode.Utils.utils.toPose2D;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;


public class Roadrunner implements subsystem {


    /**
     * the maximum translational acceleration of our robot
     */
    public final static double MAX_ROBOT_ACCELERATION = 30;

    public static final double ROBOT_SIZE = 17;
    public static final double ROBOT_RADIUS = ROBOT_SIZE / 2;
    // inches per second
    /**
     * the maximum velocity of our robot
     */
    public final static double MAX_ROBOT_VELOCITY = 30;//55;

    /**
     * the maximum angular speed of our robot
     */
    public final static double MAX_ANGULAR_VELOCITY = Math.toRadians(300);
    /**
     * the maximum angular acceleration of our robot
     */
    public final static double MAX_ANGULAR_ACCELERATION = Math.toRadians(240);

    public SampleMecanumDrive mecanumDrive;

    public double last_FrontLeft_Power = 0;
    public double last_BackRight_Power = 0;
    public double last_BackLeft_Power = 0;
    public double last_FrontRight_Power = 0;




    protected VoltageSensor batterySensor;

    protected HardwareMap hwmap;

    public Roadrunner(VoltageSensor batterySensor) {
        this.batterySensor = batterySensor;
    }

    @Override
    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;

        mecanumDrive = new SampleMecanumDrive(hwmap);

    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {

        mecanumDrive.update();
        PoseStorage.pose = fromPose2D(mecanumDrive.getPoseEstimate());

    }





    /**
     * set robot relative motor powers
     *
     * @param xSpeed    speed in robot forward direction
     * @param turnSpeed turning speed
     */
    public void robotRelative(double xSpeed, double ySpeed, double turnSpeed) {
        mecanumDrive.setWeightedDrivePower(new Pose2d(xSpeed, ySpeed, turnSpeed));
    }

    public void robotRelativeConstrained(double xSpeed, double ySpeed, double turnSpeed,
                                         double yThreshold) {
        if (Math.abs(ySpeed) < Math.abs(yThreshold)) {
            ySpeed = 0;
        }
        robotRelative(xSpeed, ySpeed, turnSpeed);
    }

    public void robotRelativeConstrained(Vector3D powers, double yThreshold) {
        robotRelativeConstrained(powers.getX(),powers.getY(),powers.getAngleRadians(),yThreshold);
    }

    /**
     * robot relative with Vector3D representation of motor powers
     * @param powers motor power vector
     */
    public void robotRelative(Vector3D powers) {
        robotRelative(powers.getX(), powers.getY(), powers.getAngleRadians());
    }

    /**
     * command the robot relative to the field
     * @param powers ideal field relative powers
     * @param robotPose current robot pose
     */
    public void fieldRelative(Vector3D powers, Vector3D robotPose) {
        Vector2d rotatedPowers = new Vector2d(powers.getX(),powers.getY());
        rotatedPowers = rotatedPowers.rotated(robotPose.getAngleRadians());
        robotRelative(new Vector3D(rotatedPowers.getX(),rotatedPowers.getY(),powers.getAngleRadians()));
    }

    public void fieldRelative(Vector3D powers) {
        Vector2d input = new Vector2d(
                powers.getX(),
                powers.getY()
        ).rotated(mecanumDrive.getPoseEstimate().getHeading());
        mecanumDrive.setWeightedDrivePower(new Pose2d(
                input.getX(),
                -input.getY(),
                -powers.getAngleRadians()
        ));
    }

    public void STOP() {
        mecanumDrive.setWeightedDrivePower(new Pose2d(0,0,0));
    }

    @Override
    public Vector3D subsystemState() {
        return fromPose2D(mecanumDrive.getPoseEstimate());
    }

    public void setPositionEstimate(Vector3D pose) {
        this.mecanumDrive.setPoseEstimate(toPose2D(pose));
    }
    public void setPositionEstimate(Pose2d pose) {
        this.mecanumDrive.setPoseEstimate(pose);
    }

    public Pose2d getPoseVelocity() {
       return this.mecanumDrive.getPoseVelocity();
    }

    public void setXPose(double x) {
        Pose2d currentEstimate = mecanumDrive.getPoseEstimate();
        Pose2d newEstimate = new Pose2d(x,currentEstimate.getY(), currentEstimate.getHeading());
        setPositionEstimate(newEstimate);
    }
    public void setYPose(double y) {
        Pose2d currentEstimate = mecanumDrive.getPoseEstimate();
        Pose2d newEstimate = new Pose2d(currentEstimate.getX(),y, currentEstimate.getHeading());
        setPositionEstimate(newEstimate);
    }
}
