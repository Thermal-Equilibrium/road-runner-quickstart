package org.firstinspires.ftc.teamcode.Controls.MIMOControls;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controls.SISOControls.PVControl;
import org.firstinspires.ftc.teamcode.Controls.SISOControls.RobustPID;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;

import homeostasis.utils.State;

import static org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients.compBotAcceleration;
import static org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients.compBotTurn;
import static org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients.compBotVelocity;
import static org.firstinspires.ftc.teamcode.Controls.Coefficients.controllerCoefficients.translationCoefficients;
import static org.firstinspires.ftc.teamcode.Utils.utils.plotVector;
import static org.firstinspires.ftc.teamcode.Utils.utils.sin_c;

public class MecanumDriveController {

	protected PVControl controllerX;
	protected PVControl controllerY;
	protected RobustPID thetaControl;

	protected Vector3D previousReferencePose = new Vector3D();

	protected ElapsedTime timer;

	MotionProfile profileX;
	MotionProfile profileY;

	boolean hasBegun = false;

	protected double powerScalar = 0;
	protected double anglePowerScalar = 0;
	protected final double rateOfAcceleration = 0.04; // TODO: tune this
	protected final double angleRateOfAcceleration = rateOfAcceleration * 4;

	/**
	 * construct Mecanum Drive Controller
	 *
	 * This initializes the individual controllers and timers.
	 */
	public MecanumDriveController() {
		this.controllerX = new PVControl(translationCoefficients);
		this.controllerY = new PVControl(translationCoefficients);
		this.thetaControl = new RobustPID(compBotTurn, 3,0.3,Math.toRadians(0.6));
		this.timer = new ElapsedTime();
	}

	/**
	 * calculate the appropriate mecanum drive motor powers
	 * @param referencePose reference position
	 * @param referenceVelocity reference velocity
	 * @param robotPose robot estimated position
	 * @param robotVelocity robot estimated velocity
	 * @return appropriate motor commands determined by the controllers.
	 */
	public Vector3D calculate(Vector3D referencePose, Vector3D referenceVelocity,
							  Vector3D robotPose, Vector3D robotVelocity) {


		if (!hasBegun) {
			timer.reset();
			hasBegun = true;
		}

		State robotX = new State(robotPose.getX(), robotVelocity.getX());
		State robotY = new State(robotPose.getY(), robotVelocity.getY());
		State referenceX = new State(referencePose.getX(), referenceVelocity.getX());
		State referenceY = new State(referencePose.getY(), referenceVelocity.getY());
		double thetaReference = referencePose.getAngleRadians();
		double robotTheta = robotPose.getAngleRadians();
		thetaControl.setReference(thetaReference);
		double sine_c_multiplier = sin_c(thetaControl.getError());
		double X_u = Range.clip(controllerX.calculate(referenceX, robotX), -1, 1) * sine_c_multiplier;
		double Y_u = Range.clip(controllerY.calculate(referenceY, robotY), -1, 1) * sine_c_multiplier;
		double Theta_u = thetaControl.calculateLinearAngle(robotTheta); // TODO: fix this

		return new Vector3D(X_u, -Y_u, Theta_u);

	}

	/**
	 * calculates the state feedback while profiling the output
	 * @param referencePose final reference position
	 * @param robotPose current robot position
	 * @param robotVelocity current robot velocity
	 * @return calculated motor command.
	 */
	public Vector3D calculateProfiled(Vector3D referencePose,
									  Vector3D robotPose, Vector3D robotVelocity) {

		checkAndGenerateProfile(referencePose, robotPose);

		MotionState stateX = profileX.get(timer.seconds());
		MotionState stateY = profileY.get(timer.seconds());

		Vector3D currentReferencePose = new Vector3D(stateX.getX(),
												   stateY.getX(),
												   referencePose.getAngleRadians());

		Vector3D referenceVelocity = new Vector3D(stateX.getV(),
												  stateY.getV(),
										  0);


		previousReferencePose = referencePose;

		return calculate(currentReferencePose,referenceVelocity,robotPose,robotVelocity);

	}





	/**
	 * checks and resets the motion profile and timer if necessary.
	 * @param referencePose the current reference pose
	 * @param robotPose the current robot pose
	 */
	public void checkAndGenerateProfile(Vector3D referencePose, Vector3D robotPose) {
		if (profileX != null &&
			profileY != null &&
			referencePose.equals(previousReferencePose)) return;
		resetTimer();
		generateMotionProfile(referencePose,robotPose);
	}



	/**
	 * generate motion profile
	 * @param referencePose target robot position
	 * @param robotPose current robot position
	 */
	public void generateMotionProfile(Vector3D referencePose, Vector3D robotPose) {
		profileX = MotionProfileGenerator.generateSimpleMotionProfile(
			new MotionState(robotPose.getX(),0,0),
			new MotionState(referencePose.getX(),0,0),
			compBotVelocity,
			compBotAcceleration
		);
		profileY = MotionProfileGenerator.generateSimpleMotionProfile(
				new MotionState(robotPose.getY(),0,0),
				new MotionState(referencePose.getY(),0,0),
				compBotVelocity,
				compBotAcceleration
		);
	}

	public Vector3D followTrajectoryToPose(Vector3D referencePose,
									   Vector3D robotPose, Vector3D robotVelocity) {
		Vector3D robotRelativePowers = calculateProfiled(referencePose,robotPose,robotVelocity);
		return robotRelativePowers.rotateBy(robotPose.getAngleDegrees());
	}

	/**
	 * check if our profile is complete
	 * @return true if the profile is complete
	 */
	public boolean isProfileComplete() {
		if (profileY == null || profileX == null) return true;
		if (profileY.duration() < timer.seconds()) return true;
		return profileX.duration() < timer.seconds();
	}

	/**
	 * check if we are complete with following our trajectory.
	 * @return true if we are complete
	 */
	public boolean followingIsComplete() {
		Dashboard.packet.put("controller x complete",controllerX.isProcessComplete());
		Dashboard.packet.put("controller y complete", controllerY.isProcessComplete());
		Dashboard.packet.put("theta control is complete",thetaControl.isComplete());
		Dashboard.packet.put("profile is complete", isProfileComplete());


		return controllerX.isProcessCompleteStrict()
				&& controllerY.isProcessCompleteStrict()
				&& (thetaControl.isComplete() || thetaControl.isBasicallyStopped() );
	}

	/**
	 * reset instance of elapsed timer
	 */
	public void resetTimer() {
		timer.reset();
	}


	public Vector3D calculateSpeedRamped(Vector3D referencePose, Vector3D robotPose, Vector3D robotVelocity) {

		if (!referencePose.equals(previousReferencePose)) {
			powerScalar = 0;
			anglePowerScalar = 0;
		}
		previousReferencePose = referencePose;

		powerScalar += rateOfAcceleration;
		anglePowerScalar += angleRateOfAcceleration;
		if (powerScalar > 1) {
			powerScalar = 1;
		}
		if (anglePowerScalar > 1) {
			anglePowerScalar = 1;
		}



		Vector3D output = calculate(referencePose, new Vector3D(), robotPose, new Vector3D());

		Dashboard.packet.put("x error", controllerX.getError().getPosition());
		Dashboard.packet.put("y error", controllerY.getError().getPosition());

		double x = Range.clip(output.getX(),-1,1);
		double y = Range.clip(output.getY(),-1,1);
		double theta = Range.clip(output.getAngleRadians(),-1,1);

		Vector3D power = new Vector3D(x, y,
				theta).rotateBy(robotPose.getAngleDegrees()).scale(powerScalar,false);
		power.setAngleRad(power.getAngleRadians() * anglePowerScalar);
		plotVector(power,"drive power",Dashboard.packet);
		return power;

	}

	public double getTime() {
		return timer.seconds();
	}



}
