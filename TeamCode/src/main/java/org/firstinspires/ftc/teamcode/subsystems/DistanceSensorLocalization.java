package org.firstinspires.ftc.teamcode.subsystems;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;

import java.util.ArrayList;

import homeostasis.Filters.SISOKalmanFilter;

import static org.firstinspires.ftc.teamcode.Utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utils.utils.fromPose2D;

@Config
public class DistanceSensorLocalization implements subsystem{

	double velocity_threshold = 3;
	public Rev2mDistanceSensor leftSensor;
	public Rev2mDistanceSensor rightSensor;
	public Rev2mDistanceSensor rearSensor;
	public final double TILE_SIZE = 24;
	public final double maximumAngle = Math.toRadians(3);

	final double leftDistanceFromCenter = 5 + (1/8.0);
	final double leftDistanceFromEdge = 12.0;
	public static double frontDistanceFromCenter = 4.5;
	final double frontDistanceFromEdge = -0.5;


	public Vector3D leftDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0.0));
	public Vector3D rearDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0.0));
	public Vector3D rightDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0.0));

	public double leftDistance = 0;
	public double rightDistance = 0;
	public double rearDistance = 0;

	Roadrunner odom;

	ArrayList<Vector3D> previousVectors = new ArrayList<>();
	double Q = 8;
	double R = 31;
	int N = 3;

	double cutoffDistanceMAX = 70;
	double minDistance = 7;

	SISOKalmanFilter estimatorX = new SISOKalmanFilter(Q,R);
	SISOKalmanFilter estimatorY = new SISOKalmanFilter(Q,R);

	public DistanceSensorLocalization(Roadrunner odom) {
		this.odom = odom;
	}

	double hz = 15;
	double delay = 1000 / hz;

	ElapsedTime timer = new ElapsedTime();

	@Override
	public void init(HardwareMap hwmap) {

		this.leftSensor = hwmap.get(Rev2mDistanceSensor.class, "LeftDistance");
		this.rightSensor = hwmap.get(Rev2mDistanceSensor.class, "RightDistance");
		this.rearSensor = hwmap.get(Rev2mDistanceSensor.class, "RearDistance");
		timer.reset();

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void update() {


		calculatePositions();

		Dashboard.packet.put("left distance sensor", leftDistance);
		Dashboard.packet.put("right distance sensor", rightDistance);
		Dashboard.packet.put("rear distance sensor", rearDistance);

	}

	@Override
	public Object subsystemState() {
		return null;
	}

	protected void readSensors() {
		this.leftDistance = leftSensor.getDistance(DistanceUnit.INCH);
		this.rearDistance = rearSensor.getDistance(DistanceUnit.INCH);
		this.rightDistance = rightSensor.getDistance(DistanceUnit.INCH);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	public void calculatePositions() {
		System.out.println("current time for scheduling is " + timer.milliseconds());
		double velocityMagnitude = fromPose2D(odom.getPoseVelocity()).distanceToPose(new Vector3D());
		Dashboard.packet.put("velocity magnitude", velocityMagnitude);
		Vector3D robotPose = odom.subsystemState();
		boolean angleOutOfRange = Math.abs(AngleWrap(robotPose.getAngleRadians())) > maximumAngle;

		if (timer.milliseconds() < delay || velocityMagnitude > velocity_threshold || angleOutOfRange) return;
		timer.reset();


		readSensors();

		//if (leftDistance >= cutoffDistanceMAX || minDistance >= leftDistance) return;
		if (rearDistance >= cutoffDistanceMAX || minDistance >= rearDistance) return;


		double x = rearDistance * Math.cos(robotPose.getAngleRadians() + rearDistanceSensorRobotRelative.getAngleRadians());
		//double y = leftDistance * Math.cos(robotPose.getAngleRadians() + leftDistanceSensorRobotRelative.getAngleRadians());


		double x_field = (TILE_SIZE * 3) - (x - frontDistanceFromEdge + frontDistanceFromCenter);
		//double y_field = -(TILE_SIZE * 3) + (y - leftDistanceFromEdge + leftDistanceFromCenter);


		//Vector3D estimatedPose = new Vector3D(x_field, y_field, robotPose.getAngleRadians());
		//plotVector(estimatedPose,"distance sensor pose estimate", Dashboard.packet);

		//drawRobotBlue(estimatedPose, Dashboard.packet);

		double xPoseEstimate = estimatorX.updateKalmanMeasurements(robotPose.getX(), x_field);
		//double yPoseEstimate = estimatorY.updateKalmanMeasurements(robotPose.getY(), estimatedPose.getY());

		// TODO FIX THIS ONCE Y POSE WORKS
		// odom.setYPose(yPoseEstimate);
		odom.setXPose(xPoseEstimate);

	}
}
