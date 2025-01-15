package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import utils.Logger;

@Autonomous(name = "Auto", group = "Opmodes")
public class Auto extends OpMode {
    Logger logger = new Logger(telemetry);

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private Follower follower;

    private PathChain builtPath;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(7.875, 65.85);

        follower.setStartingPose(startPose);

        builtPath = follower.pathBuilder()
            .addPath(
                // Line 2
                new BezierLine(
                    new Point(startPose),
                    new Point(new Pose(35, 65.85))
                )
            )
            .setTangentHeadingInterpolation()
           /* .addPath(
                    // Line 3
                    new BezierLine(
                            new Point(110.000, 110.000, Point.CARTESIAN),
                            new Point(80.000, 110.000, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()*/
                .build();
    }

    @Override
    public void start() {
        follower.followPath(builtPath, true);
    }

    @Override public void loop() {
        logger.Log("X: " + follower.getPose().getX());
        logger.Log("Y: " + follower.getPose().getY());
        logger.Log("Heading: " + follower.getPose().getHeading());
        follower.update();
    }

    @Override
    public void stop() {
        logger.close();
    }
}
