package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Ducks.SetDuckWheel;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToMidDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.NoSlideDeposit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.FieldSide;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

import static org.firstinspires.ftc.teamcode.Utils.utils.fromPose2D;
import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.ALLIANCE.BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.FieldSide.ALLIANCE.RED;

@Autonomous
public class BlueDuckAutoRR extends BaseAuto {

    Pose2d startPosition = reflect(new Pose2d(-TILE * 1.5, -TILE * 3 + 8.375,  Math.toRadians(-90)));

    Pose2d middleDeposit = reflect(new Pose2d(-TILE + 1, -2 * TILE + 3, Math.toRadians(60 + 180)));
    double depositTangent = Math.toRadians(-30.0);
    Pose2d highDeposit = reflect(new Pose2d(-TILE + 1, -2 * TILE + 3, Math.toRadians(60 + 180)));

    Pose2d lowDeposit = reflect(new Pose2d(-TILE - 2, -TILE - 12, Math.toRadians(-180+ 40)));

    double lowTangent = Math.toRadians(-30.0);


    Pose2d carouselPose = reflect(new Pose2d(-TILE * 2 - 12, -TILE * 3 + 13, Math.toRadians(180)));
    double carouselTangent = Math.toRadians(-180);

    Pose2d park = reflect(new Pose2d(-TILE * 2.5, -TILE * 1.5,Math.toRadians(0)));
    double parkTangent = Math.toRadians(-180.0);

    Trajectory goToMiddleDeposit;
    Trajectory goToHighDeposit;
    Trajectory goToLowDeposit;


    Trajectory goToCarousel;
    Trajectory goToPark;

    @Override
    public void setStartingPosition() {
        goToMiddleDeposit = roadrunnerDrive.trajectoryBuilder(startPosition, true)
                .splineToSplineHeading(middleDeposit, depositTangent)
                .build();
        goToLowDeposit = roadrunnerDrive.trajectoryBuilder(startPosition, true)
                .lineToSplineHeading(lowDeposit)
                .build();
        goToHighDeposit = roadrunnerDrive.trajectoryBuilder(startPosition, true)
                .splineToSplineHeading(highDeposit, depositTangent)
                .build();

        goToCarousel = roadrunnerDrive.trajectoryBuilder(goToMiddleDeposit.end(), false)
                .lineToSplineHeading(carouselPose, SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(100), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        goToPark = roadrunnerDrive.trajectoryBuilder(goToCarousel.end(),true)
                .lineToSplineHeading(park)
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

        actions.add(new Delay(3000));


        switch (TSEPosition) {
            case LEFT:
                actions.add(new NoSlideDeposit(robot));
                actions.add(new FollowTrajectory(robot, goToLowDeposit));

                break;
            case MIDDLE:
                actions.add(new GoToMidDeposit(robot));
                actions.add(new FollowTrajectory(robot, goToMiddleDeposit));

                break;
            case RIGHT:
                actions.add(new GoToHighDeposit(robot));
                actions.add(new FollowTrajectory(robot, goToHighDeposit));

                break;
        }


        actions.add(new DepositFreight(robot));

        actions.add(new GoToInState(robot));

        actions.add(new FollowTrajectory(robot, goToCarousel));

        actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.ON));
        actions.add(new Delay(3200));
        actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OFF));

        actions.add(new FollowTrajectory(robot, goToPark));

    }

    public static Pose2d reflect(Pose2d other) {
        return new Pose2d(other.getX(), -other.getY(), -other.getHeading());
    }
}
