package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;

import static org.firstinspires.ftc.teamcode.util.DashboardUtil.ROBOT_RADIUS;

public class utils {

    public final static double TAU = Math.PI * 2;

    /**
     * Makes sure an angle is in the range of -180 to 180
     *
     * @param angle angle we want to unwrap
     * @return the corrected angle
     */
    public static double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0* Math.PI;
        }
        while (angle> Math.PI){
            angle -= 2.0* Math.PI;
        }
        return angle;
    }

    /**
     * Makes sure an angle is in the range of -180 to 180
     * @param degrees angle we want to unwrap
     * @return the corrected angle in degrees
     */
    public static double AngleWrapDeg(double degrees) {
        while (degrees < -180) {
            degrees += 2.0 * 180;
        }
        while (degrees > 180) {
            degrees -= 2.0 * 180;
        }
        return degrees;
    }
    /**
     * squish motor power between 0 and 1
     * @param power un squished motor power
     * @return squished
     */
    public static double clipMotor(double power) {
        return Range.clip(power, -1,1);
    }

    /**
     * angle normalization from road runner
     * @param radians the angle in radians we want to normalize
     * @return the modified angle in radians
     */
    public static double normalizeAngleRR(double radians) {
        double modifiedAngle = radians % TAU;
        modifiedAngle = (modifiedAngle + TAU) % TAU;
        return modifiedAngle;
    }


    public static double normalizeAngle(double radians) {
        return AngleWrap(-normalizeAngleRR(radians));
    }


    /**
     * calculates the normalized heading error from roadrunner odometry
     * @param referenceRadians the angle we would like to be at
     * @param stateRadians the angle where we are
     * @return the normalized heading error in radians
     */
    public static double normalizedHeadingError(double referenceRadians, double stateRadians) {

        return normalizeAngle(referenceRadians - stateRadians);
    }

    /**
     * uses the normalized heading error function to get the angle difference between the target angle of two positions
     * @param referencePosition where we want to be
     * @param currentPosition where we are
     * @return the normalized angle of the two angle setpoints
     */
    public static double positionHeadingError(Vector3D referencePosition, Vector3D currentPosition) {
        return normalizedHeadingError(referencePosition.getAngleRadians(),currentPosition.getAngleRadians());
    }


    /**
     * estimate second order integral
     * @param value the current value we want to integrate
     * @param lastValue the last value we integrated
     * @param dt the delta time
     * @return the integral estimate
     */
    public static double approxSecondOrderIntegral(double value, double lastValue, double dt) {
        double sum = 0;
        sum += ((lastValue + value) / 2) * dt;
        return sum;
    }


    /**
     * estimate first order integral
     *
     * @param value the value we would like to integrate
     * @param dt    the delta time constant
     * @return the instantaneous integration
     */
    public static double approxFirstOrderIntegral(double value, double dt) {
        double sum;
        sum = value * dt;
        return sum;
    }

    public static void drawRobot(Vector3D position, TelemetryPacket packet) {

        Pose2d pose = position.toPose2d();
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);

        double x1 = pose.getX() + v.getX() / 2;
        double y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX();
        double y2 = pose.getY() + v.getY();


        packet.fieldOverlay()
                .setStroke("black")
                .strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS)
                .strokeLine(x1, y1, x2, y2);

    }

    public static void drawRobotGreen(Vector3D position, TelemetryPacket packet) {

        Pose2d pose = position.toPose2d();
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);

        double x1 = pose.getX() + v.getX() / 2;
        double y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX();
        double y2 = pose.getY() + v.getY();


        packet.fieldOverlay()
                .setStroke("green")
                .strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS)
                .strokeLine(x1, y1, x2, y2);

    }

    public static void drawRobotBlue(Vector3D position, TelemetryPacket packet) {

        Pose2d pose = position.toPose2d();
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);

        double x1 = pose.getX() + v.getX() / 2;
        double y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX();
        double y2 = pose.getY() + v.getY();


        packet.fieldOverlay()
                .setStroke("blue")
                .strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS)
                .strokeLine(x1, y1, x2, y2);

    }

    /**
     * plot a vector3d object on the dashboard
     * @param vector vector we want to plot
     * @param label what the vector is representing
     * @param packet the packet we want to push to
     */
    public static void plotVector(Vector3D vector, String label, TelemetryPacket packet) {
        packet.put(label + " x" , vector.getX());
        packet.put(label + " y" , vector.getY());
        packet.put(label + " z" , vector.getZ());
        packet.put(label + " theta" , vector.getAngleRadians());
    }

    /**
     * sin c function, traditionally used for signal processing but produces a nice shape for
     * scaling our differential drive controller to fix angle error.
     * @param x value we want to calculate sin c for.
     * @return sin_c of x.
     */
    public static double sin_c(double x) {
        if (x == 0) return 1;

        try {
            return Math.sin(x) / x;
        } catch (ArithmeticException e) {
            System.out.println("divide by zero occured, defaulting to 1");
            return 1;
        }
    }

    public static Vector3D fromPose2D(Pose2d pose2d) {
        return new Vector3D(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

}
