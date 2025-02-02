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
import java.util.Arrays;
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

    DelaySystem delaySystem;

    boolean didStateAction = false;

    List<Pose> specimenPoses = new ArrayList<>();

    int goalSpecimens = 3;
    int specimensScored = 0;
    double specimenHangX = 37;
    double specimenGap = 2;
    double startY = 65.85;

    PathChain initScorePath;
    List<List<PathChain>> cyclePaths = new ArrayList<>();

    long stateStartTime = 0;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        delaySystem = robot.delaySystem;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(7.875, 65.85);
        Pose grabPose = new Pose(10, 36);
        Pose grabConfirmPose = new Pose(7.875, 36);

        follower.setStartingPose(startPose);

        grabConfirmPath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grabPose),
                    new Point(grabConfirmPose)
                )
            )
            .setConstantHeadingInterpolation(0)
            .build();

        for (int i = 0; i < goalSpecimens; i++) {
            specimenPoses.add(new Pose(specimenHangX, startY + specimenGap * i));
        }

        initScorePath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(specimenPoses.get(0))
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        for (int i = 1; i < goalSpecimens; i++) {
            Pose lastSpecimenPose = specimenPoses.get(i - 1);

            PathChain grabPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(lastSpecimenPose),
                                    new Point(grabPose)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .build();

            PathChain scorePath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabConfirmPose),
                                new Point(specimenPoses.get(i))
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

            cyclePaths.add(new ArrayList<>(Arrays.asList(grabPath, scorePath)));
        }

        parkPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenPoses.get(specimenPoses.size() - 1)),
                                new Point(new Pose(10, 10))
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
    }

    @Override
    public void init_loop() {
        robot.Update();
        follower.update();
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
                        robot.arm.SetClawPosition(Arm.ClawPosition.Open);
                        specimensScored++;
                        delaySystem.CreateDelay(500, () -> {
                            didStateAction = false;
                            if (specimensScored < goalSpecimens) {
                                state = AutoState.DoingGrabMove;
                            }
                            else {
                                state = AutoState.Parking;
                            }
                        });
                    });
                }
                break;
            case DoingGrabMove:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.PrepareToGrabSpecimen();
                    PathChain grabPath = cyclePaths.get(specimensScored - 1).get(0);
                    follower.followPath(grabPath, true);
                }
                else if (!follower.isBusy()) {
                    didStateAction = false;
                    state = AutoState.GoingForGrab;
                }
                break;
            case GoingForGrab:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.SetClawPosition(Arm.ClawPosition.Open); //TODO: remove after new claw is printed and fourbar passthrough with open claw is allowed
                    follower.followPath(grabConfirmPath);
                }
                else if (!follower.isBusy()) {
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
                    PathChain scorePath = specimensScored == 0 ? initScorePath : cyclePaths.get(specimensScored - 1).get(1);
                    follower.followPath(scorePath, true);
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

        telemetry.addData("Specimens Scored", specimensScored);

        robot.logger.Log("X: " + follower.getPose().getX());
        robot.logger.Log("Y: " + follower.getPose().getY());

        robot.Update();
        follower.update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}
