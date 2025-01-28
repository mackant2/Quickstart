package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import components.Arm;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import utils.DelaySystem;
import utils.Robot;

@Autonomous(name = "Specimen Auto", group = "Opmodes")
public class Auto extends OpMode {
    enum AutoState {
        Idle,
        ScoringSpecimen,
        DoingGrabMove,
        GoingForGrab,
        GrabbingSpecimen,
        DoingScoreMove,
        Parking
    }

    private AutoState state = AutoState.Idle;

    Robot robot;

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private Follower follower;

    private PathChain parkPath, grabConfirmPath;

    DelaySystem delaySystem = new DelaySystem();

    boolean didStateAction = false;

    double specimenHangX = 35;

    Pose[] specimenPoses = {
        new Pose(specimenHangX, 65.85),
        new Pose(specimenHangX, 70.85)
    };

    int goalSpecimens = 2;
    int specimensScored = 0;

    List<PathChain> grabPaths = new ArrayList<>();
    List<PathChain> scorePaths = new ArrayList<>();

    long stateStartTime = 0;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        robot.arm.SetClawPosition(Arm.ClawPosition.Closed);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(7.875, 65.85);
        Pose grabPose = new Pose(10, 36);
        Pose grabConfirmPose = new Pose(7.875, 36);

        follower.setStartingPose(startPose);

        scorePaths.add(follower.pathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(startPose),
                            new Point(specimenPoses[0])
                    )
            )
            .setConstantHeadingInterpolation(0)
            .build());

        parkPath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Point(specimenPoses[specimenPoses.length - 1]),
                    new Point(new Pose(10, 10))
                )
            )
            .setConstantHeadingInterpolation(0)
            .build();

        grabConfirmPath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grabPose),
                    new Point(grabConfirmPose)
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
                    .setConstantHeadingInterpolation(0)
                    .build()
            );

            scorePaths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabConfirmPose),
                                new Point(specimenPoses[i])
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build()
            );
        }

        delaySystem.CreateDelay(1500, robot.arm::Reset);
    }

    @Override
    public void init_loop() {
        follower.update();
        delaySystem.Update();
    }

    @Override
    public void start() {
        robot.arm.PrepareToDepositSpecimen();
        state = AutoState.DoingScoreMove;
    }

    @Override public void loop() {
        switch (state) {
            case ScoringSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.HangSpecimen();
                    delaySystem.CreateDelay(1500, () -> {
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
                    state = AutoState.GoingForGrab;
                }
                break;
            case GoingForGrab:
                if (!didStateAction) {
                    didStateAction = true;
                    stateStartTime = new Date().getTime();
                    follower.followPath(grabConfirmPath);
                }
                else if (new Date().getTime() - stateStartTime >= 1000) {
                    didStateAction = false;
                    state = AutoState.GrabbingSpecimen;
                }
            case GrabbingSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.SetClawPosition(Arm.ClawPosition.Closed);
                    delaySystem.CreateDelay(3000, () -> {
                        didStateAction = false;
                        state = AutoState.DoingScoreMove;
                    });
                }
                break;
            case DoingScoreMove:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.PrepareToDepositSpecimen();
                    follower.followPath(scorePaths.get(specimensScored), true);
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

        telemetry.addData("Current Path Index", specimensScored);

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
