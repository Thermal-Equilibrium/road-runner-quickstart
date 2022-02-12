package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.NoSlideDeposit;
import org.firstinspires.ftc.teamcode.opmodes.FieldSide;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

import static org.firstinspires.ftc.teamcode.Utils.utils.fromPose2D;
import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.ALLIANCE.BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.ALLIANCE.RED;

@Autonomous
public class BlueDuckAutoRR extends BaseAuto {

    Pose2d startPosition = new Pose2d(-TILE * 1.5, TILE * 3 - 8.375,  Math.toRadians(-90));
    Pose2d depositPose = new Pose2d(-TILE, 2 * TILE, Math.toRadians(-60 - 180));
    double depositTangent = Math.toRadians(30.0);

    Pose2d carouselPose = new Pose2d(-TILE * 2, TILE * 3 + 12, Math.toRadians(-180));
    double carouselTangent = Math.toRadians(180.0);

    Pose2d park = new Pose2d(-TILE * 2.5, TILE * 1.5,Math.toRadians(0));
    double parkTangent = Math.toRadians(180.0);

    Trajectory goToDeposit;
    Trajectory goToCarousel;
    Trajectory goToPark;

    @Override
    public void setStartingPosition() {
        goToDeposit = roadrunnerDrive.trajectoryBuilder(startPosition, true)
                .splineToSplineHeading(depositPose, depositTangent)
                .build();

        goToCarousel = roadrunnerDrive.trajectoryBuilder(goToDeposit.end(), false)
                .splineToSplineHeading(carouselPose, carouselTangent)
                .build();

        goToPark = roadrunnerDrive.trajectoryBuilder(goToCarousel.end(),true)
                .splineToLinearHeading(park,parkTangent)
                .build();


        robot.setRobotPose(fromPose2D(startPosition));
    }

    @Override
    public void setVisionSettings() {
        FieldSide.alliance = BLUE;

    }

    @Override
    public void addActions() {

        // makes sure that the bucket stays out after auto init
        actions.add(new NoSlideDeposit(robot));

        actions.add(new FollowTrajectory(robot, goToDeposit));
        actions.add(new FollowTrajectory(robot, goToCarousel));
        actions.add(new FollowTrajectory(robot, goToPark));

    }
}

