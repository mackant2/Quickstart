package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

import components.Arm;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import utils.DelaySystem;
import utils.Logger;
import utils.Robot;

@Autonomous(name = "Auto", group = "Opmodes")
public class Auto extends OpMode {
    enum AutoState {
        Idle,
        DoingInitMove,
        ScoringSpecimen,
        DoingGrabMove,
        GrabbingSpecimen,
        DoingScoreMove,
        Parking
    }

    private AutoState state = AutoState.Idle;

    Robot robot;

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private Follower follower;

    private PathChain initPath, parkPath;

    DelaySystem delaySystem = new DelaySystem();

    boolean didStateAction = false;

    Pose[] specimenPoses = {
        new Pose(35, 65.85, Math.toRadians(1)),
        new Pose(35, 75.85, Math.toRadians(1))
    };

    int goalSpecimens = 2;
    int specimensScored = 0;

    List<PathChain> grabPaths = new ArrayList<>();
    List<PathChain> scorePaths = new ArrayList<>();

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        robot.arm.Reset();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(7.875, 65.85);
        Pose grabPose = new Pose(15, 24, Math.PI);

        follower.setStartingPose(startPose);

        initPath = follower.pathBuilder()
            .addPath(
                // Line 2
                new BezierLine(
                    new Point(startPose),
                    new Point(specimenPoses[0])
                )
            )
            .setTangentHeadingInterpolation()
            .build();

        parkPath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Point(specimenPoses[specimenPoses.length - 1]),
                    new Point(new Pose(10, 10))
                )
            )
            .setConstantHeadingInterpolation(0)
            .build();

        for (int i = 1; i < goalSpecimens; i++) {
            Pose specimenPose = specimenPoses[i - 1];

            grabPaths.add(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(specimenPose),
                                    new Point(grabPose)
                            )
                    )
                    .setLinearHeadingInterpolation(specimenPose.getHeading(), grabPose.getHeading())
                    .build()
            );

            scorePaths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(specimenPose)
                        )
                )
                .setLinearHeadingInterpolation(grabPose.getHeading(), specimenPose.getHeading())
                .build()
            );
        }
    }

    @Override
    public void start() {
        robot.arm.GoToSpecimenPosition();
        delaySystem.CreateDelay(1000, () -> state = AutoState.DoingInitMove);
    }

    @Override public void loop() {
        switch (state) {
            case DoingInitMove:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(initPath, true);
                }
                else if (!follower.isBusy()) {
                    didStateAction = false;
                    state = AutoState.ScoringSpecimen;
                }
                break;
            case ScoringSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.HangSpecimen();
                    delaySystem.CreateDelay(1000, () -> {
                        robot.arm.ReleaseSpecimen();
                        specimensScored++;
                        didStateAction = false;
                        if (specimensScored < goalSpecimens) {
                            state = AutoState.DoingGrabMove;
                        }
                        else {
                            state = AutoState.Parking;
                        }
                    });
                }
                break;
            case DoingGrabMove:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.PrepareToGrabSpecimen();
                    follower.followPath(grabPaths.get(specimensScored - 1), true);
                }
                else if (!follower.isBusy()) {
                    didStateAction = false;
                    state = AutoState.GrabbingSpecimen;
                }
                break;
            case GrabbingSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.RotateFourBar(Arm.FourBarPosition.Specimen);
                    delaySystem.CreateDelay(500, () -> {
                        robot.arm.SetClawPosition(Arm.ClawPosition.Closed);
                        delaySystem.CreateDelay(1000, () -> {
                            didStateAction = false;
                            state = AutoState.DoingScoreMove;
                        });
                    });
                }
                break;
            case DoingScoreMove:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.PrepareToDepositSpecimen();
                    follower.followPath(scorePaths.get(specimensScored - 1), true);
                }
                else if (!follower.isBusy()) {
                    didStateAction = false;
                    state = AutoState.ScoringSpecimen;
                }
                break;
            case Parking:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(parkPath, true);
                }
                else if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }

        robot.logger.Log("X: " + follower.getPose().getX());
        robot.logger.Log("Y: " + follower.getPose().getY());
        robot.logger.Log("Heading: " + follower.getPose().getHeading());
        follower.update();
        delaySystem.Update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}
