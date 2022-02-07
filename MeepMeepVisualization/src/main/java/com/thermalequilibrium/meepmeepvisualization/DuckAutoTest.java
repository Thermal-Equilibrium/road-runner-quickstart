package com.thermalequilibrium.meepmeepvisualization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DuckAutoTest {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);
		double TILE = 24;

		Pose2d depositPose = new Pose2d(-TILE, -2 * TILE, Math.toRadians(60 + 180));
		double depositTangent = Math.toRadians(30);

		Pose2d carouselPose = new Pose2d(-TILE * 2, -TILE * 3 + 12, Math.toRadians(180));
		double carouselTangent = Math.toRadians(180);

		Pose2d park = new Pose2d(-TILE * 2.5, -TILE * 1.5,Math.toRadians(0));
		double parkTangent = Math.toRadians(180);


		RoadRunnerBotEntity RedBot = new DefaultBotBuilder(meepMeep).setDimensions(12,18)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 30, Math.toRadians(180), Math.toRadians(180), 11)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(new Pose2d(-TILE * 1.5, -TILE * 3 + 8.375,  Math.toRadians(-90)))
								.setReversed(true)
								.splineToSplineHeading(depositPose, depositTangent)
								.setReversed(false)
								.splineToSplineHeading(carouselPose,carouselTangent)
								.setReversed(true)
								.splineToLinearHeading(park, parkTangent)
								.build()
				);

		RoadRunnerBotEntity BlueBot = new DefaultBotBuilder(meepMeep).setDimensions(12,18).setColorScheme(new ColorSchemeBlueLight())
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 30, Math.toRadians(180), Math.toRadians(180), 11)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(new Pose2d(-TILE * 1.5, TILE * 3 - 8.375,  Math.toRadians(90)))
								.setReversed(true)
								.splineToSplineHeading(reflect(depositPose), -depositTangent)
								.setReversed(false)
								.splineToSplineHeading(reflect(carouselPose),-carouselTangent)
								.setReversed(true)
								.splineToLinearHeading(reflect(park), -parkTangent)
								.build()
				);
		meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(RedBot)
				.addEntity(BlueBot)
				.start();
	}

	public static Pose2d reflect(Pose2d other) {
		return new Pose2d(other.getX(), -other.getY(), -other.getHeading());
	}
}
