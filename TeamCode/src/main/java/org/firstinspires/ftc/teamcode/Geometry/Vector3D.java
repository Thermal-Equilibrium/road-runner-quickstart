
package org.firstinspires.ftc.teamcode.Geometry;



import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.Utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utils.utils.approxFirstOrderIntegral;
import static org.firstinspires.ftc.teamcode.Utils.utils.normalizedHeadingError;

public class Vector3D {

    public boolean approach_acceleration_quicker = false;
    protected double x, y, z;


    private double timeOutDistance = 0.75;

    private final double timeOutRadians = Math.toRadians(2.4);

    public Angle angle;

    /**
     * constructor for position
     *
     * @param x       x position
     * @param y       y position
     * @param radians in radians
     */
    public Vector3D(double x, double y, double radians) {
        this.x = x;
        this.y = y;
        this.angle = new Angle(radians);
    }

    public Vector3D(double x, double y, double radians, boolean approach_acceleration_quicker) {
        this.x = x;
        this.y = y;
        this.angle = new Angle(radians);
        this.approach_acceleration_quicker = approach_acceleration_quicker;
    }

    public Vector3D(double x, double y, double z, double radians, double timeOutDistance) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.angle = new Angle(radians);
        this.timeOutDistance = timeOutDistance;

    }

    public Vector3D(double x, double y, double radians, double timeOutDistance) {
        this.x = x;
        this.y = y;
        this.angle = new Angle(radians);
        this.timeOutDistance = timeOutDistance;
    }

    public Vector3D() {
        this.x = 0;
        this.y = 0;
        this.angle = new Angle(0);
    }

    public Vector3D(Pose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.angle = new Angle(pose.getRotation().getRadians());
    }

    /**
     construction of ftclib position
     */
    public Vector3D(Translation2d translation, Rotation2d rotation) {
        this.x = translation.getX();
        this.y = translation.getY();
        this.angle = new Angle(rotation.getRadians());
    }

    /**
     * construction from roadrunner
     * @param poseEstimate roadrunner pose2d
     */
    public Vector3D(com.acmerobotics.roadrunner.geometry.Pose2d poseEstimate) {
        this.x = poseEstimate.getX();
        this.y = poseEstimate.getY();
        this.angle = new Angle(poseEstimate.getHeading());
    }

    /**
     * returns distance from this position to pos
     *
     * @param pos the other position
     * @return distance to pos
     */
    public double distanceToPose(Vector3D pos) {
        return Math.sqrt(Math.pow(pos.x - this.x, 2) + Math.pow(pos.y - this.y, 2));
    }

    /**
     * get the angle from this position to another position
     *
     * @param pos the other position we are getting the angle towards
     * @return the angle between the two position
     */
    public double returnHeadingToPose(Vector3D pos) {
        return Math.atan2(pos.y - this.y, pos.x - this.x);
    }

    /**
     * get the angle of the vector in degrees.
     *
     * @return degrees
     */
    public double getAngleDegrees() {
        return angle.getDegrees();
    }

    /**
     * get the angle of the vector in radians.
     *
     * @return radians
     */
    public double getAngleRadians() {
        return angle.getRadians();
    }

    /**
     * get the x component of the vector
     *
     * @return the x component of the vector
     */
    public double getX() {
        return x;
    }


    public double getZ() {
        return z;
    }

    /**
     * set the x component of the vector
     *
     * @param x x component of the vector
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * get the y component of the vector
     *
     * @return the y component of the vector
     */
    public double getY() {
        return y;
    }

    /**
     * set the y component of the vector
     *
     * @param y y component of the vector
     */
    public void setY(double y) {
        this.y = y;
    }


    /**
     * set pose based on primative translation and rotation calsses
     *
     * @param translation An FTClib translation object containing x,y coordinates
     * @param rotation    An FTClib rotation object containing an angle.
     */
    public void setPoseFTClib(Translation2d translation, Rotation2d rotation) {
        this.x = translation.getX();
        this.y = translation.getY();
        this.angle = new Angle(rotation.getRadians());
    }

    /**
     * set position based on ftclib pose2d
     *
     * @param pose the Ftclib Pose2d object
     */
    public void setPose2d(Pose2d pose) {
        this.x = pose.getTranslation().getX();
        this.y = pose.getTranslation().getY();
        this.angle = new Angle(pose.getRotation().getRadians());
    }

    /**
     * sets position based on roadrunner pose2d
     * @param pose roadrunner position estimate
     */
    public void setPose2dRoadRunner(com.acmerobotics.roadrunner.geometry.Pose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.angle.setRadians(pose.getHeading());
    }

    /**
     * calculates the midpoint of this point and the other point
     * @param other the other point
     * @return the mid point
     */
    public Vector3D midpoint(Vector3D other) {
        double x = (this.x + other.x) / 2;
        double y = (this.y + other.y) / 2;
        double angleRad = this.getAngleRadians();
        return new Vector3D(x, y, angleRad);

    }

    /**
     * set the angle of the vector as radians
     *
     * @param rad the angle
     */
    public void setAngleRad(double rad) {
        this.angle.setRadians(rad);
    }


    /**
     * add two vectors together
     *
     * @param other the other vector we are adding
     * @return the sum of the two vectors
     */
    public Vector3D plus(Vector3D other) {
        return new Vector3D(other.x + this.x, other.y + this.y, AngleWrap(other.getAngleRadians() + this.angle.getRadians()));
    }


    /**
     * convert the vector to roadrunner Pose2d
     *
     * @return Pose2D
     */
    public com.acmerobotics.roadrunner.geometry.Pose2d toPose2d() {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(this.x, this.y, this.angle.radians);
    }

    public Pose2d toBetterPose2d() {
        return new Pose2d(this.x,this.y,new Rotation2d(this.angle.radians));
    }

    /**
     * return error for where we are where the parent class is the reference
     * @param state where we are
     * @return the error between the reference and the state
     */
    public Vector3D getError(Vector3D state) {

        Vector3D error = new Vector3D(0, 0, 0);
        error.setX(this.x - state.getX());
        error.setY(this.y - state.getY());
        error.setAngleRad(normalizedHeadingError(this.angle.radians, state.getAngleRadians()));

        return error;

    }

    /**
     * used to offset this vector from the offset, such as zeroing a position
     *
     * @param offset the vector we are offsetting
     * @return the offset vector
     */
    public Vector3D offsetVector(Vector3D offset) {

        Vector3D error = new Vector3D(0, 0, 0);
        error.setX(this.x - offset.getX());
        error.setY(this.y - offset.getY());
        error.setAngleRad(normalizedHeadingError(offset.getAngleRadians(), this.angle.radians));

        return error;

    }

    /**
     * converts the vector to a java 2d array
     *
     * @return the 2d array
     */
    public double[][] vectorToMatrix() {
        return new double[][]{{this.x, 0, 0}, {0, this.y, 0}, {0, 0, this.getAngleRadians()}};
    }

    /**
     * scale the vector by a scaler
     *
     * @param scaler     the amount we are scaling the entire vector by
     * @param scaleAngle if we are scaling our angle or just x, y.  true for all 3, false for just x, y
     * @return the scaled vector
     */
    public Vector3D scale(double scaler, boolean scaleAngle) {
        Vector3D mult = new Vector3D(this.x, this.y, this.angle.radians);
        mult.setX(mult.x * scaler);
        mult.setY(mult.y * scaler);
        if (scaleAngle) {
            mult.setAngleRad(this.angle.radians * scaler);
        }

        return mult;
    }


    /**
     * similar to the previous method but with a more accurate assumption about the change in values over time
     *
     * @param other     the data we are integrating
     * @param deltaTime the time constant
     * @return the integrated vector
     */
    public Vector3D integrateFirstOrder(Vector3D other, double deltaTime) {
        // change after integration
        double xDeltaPrime = approxFirstOrderIntegral(other.x, deltaTime);
        double yDeltaPrime = approxFirstOrderIntegral(other.y, deltaTime);
        double thetaDeltaPrime = approxFirstOrderIntegral(other.getAngleRadians(), deltaTime);

        double xPrime = this.x + xDeltaPrime;
        double yPrime = this.y + yDeltaPrime;
        double thetaPrime = AngleWrap(this.getAngleRadians() + thetaDeltaPrime);

        return new Vector3D(xPrime, yPrime, thetaPrime);
    }

    /**
     * for cases when we dont want to perform integration on the angle and just our X, Y vector we use this method
     *
     * @param other     data we are integrating
     * @param deltaTime time constant
     * @return the integrated vector with the angle set to 0 radians
     */
    public Vector3D integrateForgoAngle(Vector3D other, double deltaTime) {
        Vector3D result = integrateFirstOrder(other, deltaTime);
        result.setAngleRad(0);
        return result;
    }

    /**
     * print the vectors string
     */
    public void printVector() {
        System.out.println(toString());
    }


    /**
     * get the vector as a string
     *
     * @return the string of the vector
     */
    public String toString() {
        return "x:" + x + " y: " + y + " radians: " + getAngleRadians();
    }


    /**
     * check if our vector is within tolerances
     *
     * @param other the place we want to be
     * @return if we are within tolerance
     */
    public boolean vectorIsWithinTimeoutDistance(Vector3D other) {
        //return Math.abs(AngleWrap(other.getAngleRadians() - this.getAngleRadians())) <= timeOutRadians;

        return (distanceToPose(other) <= timeOutDistance) && Math.abs(AngleWrap(other.getAngleRadians() - this.getAngleRadians())) <= timeOutRadians;
    }


    /**
     * vector multiplication
     *
     * @param other the vector we are multiplying
     * @return the new product
     */
    public Vector3D multiply(Vector3D other) {
        Vector3D result = new Vector3D(this.x, this.y, this.angle.radians);
        result.setX(this.x * other.x);
        result.setY(this.y * other.y);
        result.setAngleRad(this.angle.radians * other.angle.radians);
        return result;
    }

    /**
     * add two Vectors
     *
     * @param other the other vector we are adding
     * @return the vector sum
     */
    public Vector3D add(Vector3D other) {
        return new Vector3D(this.x + other.x, this.y + other.y, this.angle.radians + other.angle.radians);
    }


    /**
     * given a maximum and minimum Vector, output a new vector that clamps the top and bottom bounds to the min and max
     *
     * @param minimum the minimum values of the vector
     * @param maximum the maximum values of the vector
     * @return the clamped vector
     */
    public Vector3D conformToMinAndMax(Vector3D minimum, Vector3D maximum) {
        double newX = x;
        double newY = y;
        double newRadians = getAngleRadians();

        if (newX > maximum.x) {
            newX = maximum.x;
        }
        if (newX < minimum.x) {
            newX = minimum.x;
        }
        if (newY < minimum.y) {
            newY = minimum.y;
        }
        if (newY > maximum.y) {
            newY = maximum.y;
        }

        return new Vector3D(newX, newY, newRadians);

    }

    /**
     * get the distance it is okay to time out at
     */
    public double getTimeOutDistance() {
        return timeOutDistance;
    }

    /**
     * are we equal to another vector
     *
     * @param other the other vector we are comparing
     * @return the result
     */
    public boolean equals(Vector3D other) {
        return this.x == other.getX() && this.y == other.getY() && this.getAngleRadians() == other.getAngleRadians();
    }

    /**
     * get the absolute value of each individual component of the vector
     *
     * @return the 'absolute' vector
     */
    public Vector3D abs() {
        return new Vector3D(Math.abs(x), Math.abs(y), Math.abs(getAngleRadians()));
    }

    /**
     * reflect the vector across the x axis (y = -y)
     *
     * useful for making opposite autonomous routines
     *
     * @return reflected vector
     */
    public Vector3D reflect() {
        return new Vector3D(this.x,-this.y,-this.angle.radians);
    }

    /**
     * check if the vectors length is zero
     * @return true if the length is zero
     */
    public boolean isZero() {
        return x == 0 && y == 0 & angle.radians == 0;
    }


    /**
     * rotate the vector by a rotation matrix
     * @param theta rotation value
     * @return the rotated vector
     */
    public Vector3D rotateBy(double theta) {
        Vector2d rotated = new Vector2d(x,y);
        rotated = rotated.rotateBy(theta);
        return new Vector3D(rotated.getX(),rotated.getY(),angle.radians);
    }

    /**
     * second order integration of a pose
     * @param delta pose
     * @return integrated pose
     */
    public Vector3D poseExponential(Vector3D delta) {
        double deltaTheta = delta.getAngleRadians();
        double theta = angle.radians;
        double cosTerm;
        double sinTerm;

        if (Math.abs(deltaTheta) <= 1e-3) {
            sinTerm = 1.0 - deltaTheta * deltaTheta / 6.0;
            cosTerm = deltaTheta / 2.0;
        } else {
            sinTerm = sin(deltaTheta) / deltaTheta;
            cosTerm = (1 - cos(deltaTheta)) / deltaTheta;
        }
        double xTransform = sinTerm * delta.getX() - cosTerm * delta.getY();
        double yTransform = cosTerm * delta.getX() + sinTerm * delta.getX();

        Vector3D integrated = new Vector3D(xTransform,yTransform,deltaTheta);

        return this.plus(integrated);
    }



}
