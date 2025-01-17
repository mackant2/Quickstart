package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import utils.Logger;
import utils.Robot;

@Autonomous(name = "Auto", group = "Opmodes")
public class Auto extends OpMode {
    Robot robot;

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private Follower follower;

    private PathChain initPath, grabPath, scorePath;

    private String state = "init";

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(7.875, 65.85);
        Pose scorePose = new Pose(38, 65.85, Math.toRadians(1));
        Pose grabPose = new Pose(12, 30, Math.PI);

        follower.setStartingPose(startPose);

        initPath = follower.pathBuilder()
            .addPath(
                // Line 2
                new BezierLine(
                    new Point(startPose),
                    new Point(scorePose)
                )
            )
            .setTangentHeadingInterpolation()
                .build();

        grabPath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Point(scorePose),
                    new Point(grabPose)
                )
            )
            .setLinearHeadingInterpolation(scorePose.getHeading(), grabPose.getHeading())
                .build();

        scorePath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grabPose),
                    new Point(scorePose)
                )
            )
            .setLinearHeadingInterpolation(grabPose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void start() {
        follower.followPath(initPath, true);
    }

    @Override public void loop() {
        switch (state) {
            case "init":

                follower.followPath(initPath, true);
                state = "grabbing off wall";
                break;
            case "grabbing off wall":
                if (!follower.isBusy()) {
                    follower.followPath(grabPath, true);
                    state = "scoring";
                }
                break;
            case "scoring":
                if (!follower.isBusy()) {
                    follower.followPath(scorePath, true);
                    state = "grabbing off wall";
                }
                break;
        }

        robot.logger.Log("X: " + follower.getPose().getX());
        robot.logger.Log("Y: " + follower.getPose().getY());
        robot.logger.Log("Heading: " + follower.getPose().getHeading());
        follower.update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}
