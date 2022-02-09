package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

public class RedDuckAutoRR extends BaseAuto {

	Pose2d depositPose = new Pose2d(-TILE, -2 * TILE, Math.toRadians(60 + 180));
	double depositTangent = Math.toRadians(30);

	Pose2d carouselPose = new Pose2d(-TILE * 2, -TILE * 3 + 12, Math.toRadians(180));
	double carouselTangent = Math.toRadians(180);

	Pose2d park = new Pose2d(-TILE * 2.5, -TILE * 1.5,Math.toRadians(0));
	double parkTangent = Math.toRadians(180);

	@Override
	public void setStartingPosition() {

	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {

	}
}
