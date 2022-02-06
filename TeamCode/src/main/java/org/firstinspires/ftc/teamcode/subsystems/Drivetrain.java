package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;

import homeostasis.systems.DcMotorPlant;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;


public class Drivetrain implements subsystem {


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

    public DcMotorEx FrontLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;

    public double last_FrontLeft_Power = 0;
    public double last_BackRight_Power = 0;
    public double last_BackLeft_Power = 0;
    public double last_FrontRight_Power = 0;

    public DcMotorPlant leftMotorSys;
    public DcMotorPlant rightMotorSys;


    protected VoltageSensor batterySensor;

    protected HardwareMap hwmap;

    public Drivetrain(VoltageSensor batterySensor) {
        this.batterySensor = batterySensor;
    }

    @Override
    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;


        FrontLeft = hwmap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hwmap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hwmap.get(DcMotorEx.class, "BackLeft");
        BackRight = hwmap.get(DcMotorEx.class, "BackRight");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (isCompBot) {
            FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
            BackLeft.setDirection(DcMotorEx.Direction.REVERSE);
            BackRight.setDirection(DcMotorEx.Direction.FORWARD);

        } else {
            FrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            FrontRight.setDirection(DcMotorEx.Direction.REVERSE);
            BackLeft.setDirection(DcMotorEx.Direction.FORWARD);
            BackRight.setDirection(DcMotorEx.Direction.REVERSE);
        }



    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {

    }


    /**
     * set individual drive motors
     * @param fl front left power
     * @param fr front right power
     * @param bl back left power
     * @param br back right power
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {

        if (fl != last_FrontLeft_Power) {
            this.FrontLeft.setPower(fl);
        }
        if (fr != last_FrontRight_Power) {
            this.FrontRight.setPower(fr);
        }
        if (bl != last_BackLeft_Power) {
            this.BackLeft.setPower(bl);
        }
        if (br != last_BackRight_Power) {
            this.BackRight.setPower(br);
        }
        last_FrontLeft_Power = fl;
        last_FrontRight_Power = fr;
        last_BackLeft_Power = bl;
        last_BackRight_Power = br;

    }


    /**
     * set robot relative motor powers
     *
     * @param xSpeed    speed in robot forward direction
     * @param turnSpeed turning speed
     */
    public void robotRelative(double xSpeed, double ySpeed, double turnSpeed) {
        ySpeed *= 1.1;
        xSpeed = Range.clip(xSpeed,-1,1);
        ySpeed = Range.clip(ySpeed, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);
        double frontLeftPower = (xSpeed + ySpeed + turnSpeed);
        double backLeftPower = (xSpeed - ySpeed + turnSpeed);
        double frontRightPower = (xSpeed - ySpeed - turnSpeed);
        double backRightPower = (xSpeed + ySpeed - turnSpeed);
        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
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

    public void STOP() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    public Object subsystemState() {
        return null;
    }


}
