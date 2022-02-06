package com.thermalequilibrium.meepmeepvisualization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualization {

	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);
		double TILE = 24;
		Pose2d depositPosition = new Pose2d(+ 2,-TILE * 2 + 4 ,Math.toRadians(-60));
		double depositTangent = Math.toRadians(120);
		Pose2d intakePosition1 = new Pose2d(10, -TILE * 3 + 8.375,0);
		double intakePosition1Tangent = Math.toRadians(330);
		Pose2d intakePosition2 = new Pose2d(60, -TILE * 3 + 8,0);
		double intakePosition2Tangent = Math.toRadians(0);
		Pose2d intakePosition3 = new Pose2d(intakePosition2.getX() + 1, intakePosition2.getY(), Math.toRadians(-10));
		double intakePosition3Tangent = Math.toRadians(-90);
		Pose2d intakePosition4 = new Pose2d(intakePosition2.getX() - 5, intakePosition2.getY(), Math.toRadians(0));
		double intakePosition4Tangent = Math.toRadians(0);

		Pose2d exitWarehouse = new Pose2d(intakePosition1.getX(), intakePosition2.getY());
		double exitWareHouseTangent = Math.toRadians(180);
		double cycleEndTangent = Math.toRadians(330 - 180);



		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(12,18)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 30, Math.toRadians(180), Math.toRadians(180), 11)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(new Pose2d(TILE / 2.0, -TILE * 3 + 8.375,  Math.toRadians(-90)))
								.setReversed(true)
								.splineToSplineHeading(depositPosition, depositTangent)
								.setReversed(false)
								.splineToLinearHeading(intakePosition1, intakePosition1Tangent)
								.splineToSplineHeading(intakePosition2, intakePosition2Tangent)
								.splineToLinearHeading(intakePosition3, intakePosition3Tangent)
								.setReversed(true)
								.splineToLinearHeading(intakePosition4,intakePosition4Tangent)
								.splineToLinearHeading(exitWarehouse, exitWareHouseTangent)
								.splineToLinearHeading(depositPosition, cycleEndTangent)
								.build()
				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}

}