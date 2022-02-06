package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controls.Coefficients.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients;
import org.firstinspires.ftc.teamcode.Controls.SISOControls.RobustPID;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;


/**
 * Aims the robot towards a desired position.
 */
public class AimAtPoint implements action {

    double allowedTimeSeconds = 2;
    Vector3D targetPosition;
    double error;
    boolean reversed;
    double angleOffset;
    ElapsedTime timeout = new ElapsedTime();

    double max_power = 0.7;
    private final Robot robot;
    private boolean isWithinTolerance = false;
    private final boolean reverseAngle;

    private RobustPID controller;


    /**
     * turn the robot to aim at a target point
     *
     * @param robot       robot class
     * @param targetPoint target point to aim at
     * @param reversed    is true if we should aim but at 180 degrees
     * @param angleOffset is the amount we add to the target angle to account for curved shooters and such
     */
    public AimAtPoint(Robot robot, Vector3D targetPoint, boolean reversed, double angleOffset, boolean reverseAngle) {
        this.robot = robot;
        this.targetPosition = targetPoint;
        this.reversed = reversed;
        this.angleOffset = angleOffset;
        this.reverseAngle = reverseAngle;
        initializeController();

    }

    public AimAtPoint(Robot robot, Vector3D targetPoint) {
        this.robot = robot;
        this.targetPosition = targetPoint;
        this.reversed = false;
        this.angleOffset = 0;
        this.reverseAngle = false;
        initializeController();
    }

    public AimAtPoint(Robot robot, Vector3D targetPoint, boolean reverseAngle, boolean reversed) {
        this.robot = robot;
        this.targetPosition = targetPoint;
        this.reversed = reversed;
        this.angleOffset = 0;
        this.reverseAngle = reverseAngle;
        initializeController();
    }

    @Override
    public void startAction() {

        if (reverseAngle) {
            this.targetPosition = this.targetPosition.reflect();
            this.angleOffset = -angleOffset;
        }
        timeout.reset();
    }

    @Override
    public void runAction() {
        Vector2d difference = targetPosition.toPose2d().vec().minus(robot.getRobotPose().toPose2d().vec());
        double theta;

        if (reversed) {
            theta = (difference.angle() - Math.PI) + angleOffset;
        } else {
            theta = (difference.angle()) + angleOffset;
        }

        this.controller.setReference(theta);
        double power = Range.clip(this.controller.calculateLinearAngle(robot.getRobotPose().getAngleRadians()), -max_power, max_power);
        error = this.controller.getError();
        robot.driveTrain.robotRelative(0, 0, power);

        if (this.controller.isComplete() || timeout.seconds() > allowedTimeSeconds) {
            isWithinTolerance = true;
        }

    }

    @Override
    public void stopAction() {
        robot.driveTrain.STOP();
    }

    @Override
    public boolean isActionComplete() {
        return isWithinTolerance;
    }

    @Override
    public boolean isActionPersistent() {
        return true;
    }

    @Override
    public boolean isAMultipleAction() {
        return false;
    }

    public void initializeController() {
        PIDFCoefficients coefficients;
        if (isCompBot) {
            coefficients = controllerCoefficients.compBotTurn;
        } else {
            coefficients = controllerCoefficients.protoBotTurn;
        }
        this.controller = new RobustPID(coefficients, 0, 3, 0.004, Math.toRadians(1));

    }
}
