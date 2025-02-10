package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import components.Arm;
import components.Intake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import utils.DelaySystem;
import utils.Robot;

@Autonomous(name = "Specimen Auto", group = "Opmodes")
public class SpecimenAuto extends OpMode {
    enum AutoState {
        Idle,
        ScoringSpecimen,
        MovingToIntake,
        IntakingSample,
        DepositingSample,
        DoingGrabMove,
        GoingForGrab,
        GrabbingSpecimen,
        MovingToSub,
        Parking
    }

    private AutoState state = AutoState.Idle;

    Robot robot;

    private final double robotWidth = 12.375;
    private final double robotLength = 16.125;

    private Follower follower;

    private PathChain parkPath, grabConfirmPath;

    DelaySystem delaySystem;

    boolean didStateAction = false;

    List<Pose> specimenPoses = new ArrayList<>();

    int goalSpecimens = 2;
    int specimensScored = 0;
    int samplesDeposited = 0;
    double specimenHangX = 37;
    double specimenGap = 5;
    double startY = 65.8125;

    PathChain initScorePath;
    List<List<PathChain>> cyclePaths = new ArrayList<>();
    List<PathChain> intakePaths = new ArrayList<>();

    double wallX = 10.5;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        delaySystem = robot.delaySystem;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        double grabY = 40;

        Pose startPose = new Pose(8.0625, 65.85);
        Pose grabPose = new Pose(wallX + 2, grabY);
        Pose grabConfirmPose = new Pose(wallX - 5, grabY);

        follower.setStartingPose(startPose);
        follower.holdPoint(startPose);

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
            specimenPoses.add(new Pose(specimenHangX + 5, startY + specimenGap * i));
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

        for (int i = 0; i < goalSpecimens - 2; i++) {
            Pose pathStart = i == 0 ? specimenPoses.get(0) : new Pose(24, 50 - (i - 1) * 6);
            Pose pathEnd = new Pose(24, 50, Math.toRadians(135));

            intakePaths.add(
                    follower.pathBuilder()
                            .addPath(
                                    i == 0 ?
                                        new BezierCurve(
                                            new Point(pathStart),
                                            new Point(10, 62),
                                            new Point(pathEnd)
                                        )
                                        :
                                        new BezierLine(
                                            new Point(pathStart),
                                            new Point(pathEnd)
                                        )
                            )
                            .setLinearHeadingInterpolation(pathStart.getHeading(), pathEnd.getHeading())
                            .build()
            );
        }

        for (int i = 1; i < goalSpecimens; i++) {
            PathChain grabPath;

            if (goalSpecimens > 2) {
                grabPath = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        i == 1 ? intakePaths.get(intakePaths.size() - 1).getPath(0).getPoint(0) : new Point(specimenPoses.get(i - 1)),
                                        //Control point that bends path to avoid getting stuck on corner of sub
                                        new Point(10, 62),
                                        new Point(grabPose)
                                )
                        )
                        .setConstantHeadingInterpolation(0)
                        .build();
            }
            else {
                grabPath = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(specimenPoses.get(0)),
                                        //Control point that bends path to avoid getting stuck on corner of sub
                                        new Point(10, 62),
                                        new Point(grabPose)
                                )
                        )
                        .setConstantHeadingInterpolation(0)
                        .build();
            }

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
                        new BezierCurve(
                                new Point(specimenPoses.get(specimenPoses.size() - 1)),
                                new Point(10, 62),
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

        telemetry.addData("Specimens Scored", specimensScored);
        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
    }

    @Override
    public void start() {
        robot.arm.RunPreset(Arm.Presets.PRE_SPECIMEN_DEPOSIT);
        state = AutoState.MovingToSub;
    }

    void TransferToState(AutoState newState) {
        didStateAction = false;
        state = newState;
    }

    @Override public void loop() {
        switch (state) {
            case ScoringSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.RunPreset(Arm.Presets.SPECIMEN_DEPOSIT);
                    delaySystem.CreateDelay(1500, () -> {
                        robot.arm.SetClawPosition(Arm.ClawPosition.Open);
                        specimensScored++;
                        delaySystem.CreateDelay(500, () -> {
                            if (specimensScored < goalSpecimens) {
                                if (specimensScored == 1 && goalSpecimens > 2) {
                                    TransferToState(AutoState.MovingToIntake);
                                }
                                else {
                                    TransferToState(AutoState.DoingGrabMove);
                                }

                            }
                            else {
                                TransferToState(AutoState.Parking);
                            }
                        });
                    });
                }
                break;
            case MovingToIntake:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(intakePaths.get(samplesDeposited), true);
                    delaySystem.CreateConditionalDelay(
                            () -> !follower.isBusy(),
                            () -> {
                                robot.intake.ExtendTo(Intake.ExtenderPosition.OUT);
                                delaySystem.CreateConditionalDelay(
                                        () -> robot.intake.GetExtenderPosition() >= Intake.ExtenderPosition.OUT,
                                        () -> delaySystem.CreateDelay(5000, () -> TransferToState(AutoState.IntakingSample))
                                );
                            }
                    );
                }
                break;
            case IntakingSample:
                if (!didStateAction) {
                    TransferToState(AutoState.DepositingSample);
                }
                break;
            case DepositingSample:
                if (!didStateAction) {
                    samplesDeposited++;
                    if (samplesDeposited < goalSpecimens - 2) {
                        TransferToState(AutoState.MovingToIntake);
                    }
                    else {
                        TransferToState(AutoState.DoingGrabMove);
                    }
                }
                break;
            case DoingGrabMove:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.RunPreset(Arm.Presets.SPECIMEN_GRAB);
                    PathChain grabPath = cyclePaths.get(specimensScored - 1).get(0);
                    follower.followPath(grabPath);
                    delaySystem.CreateConditionalDelay(
                            () -> follower.getPose().getX() < wallX + 2.5,
                            () -> delaySystem.CreateDelay(2000, () -> TransferToState(AutoState.GoingForGrab))
                    );
                }
                break;
            case GoingForGrab:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(grabConfirmPath, false);
                    delaySystem.CreateConditionalDelay(
                            () -> follower.getPose().getX() < wallX + 0.5,
                            () -> TransferToState(AutoState.GrabbingSpecimen)
                    );
                }
            case GrabbingSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.SetClawPosition(Arm.ClawPosition.Closed);
                    delaySystem.CreateDelay(3000, () -> {
                        didStateAction = false;
                        state = AutoState.MovingToSub;
                    });
                }
                break;
            case MovingToSub:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.RunPreset(Arm.Presets.PRE_SPECIMEN_DEPOSIT);
                    PathChain scorePath = specimensScored == 0 ? initScorePath : cyclePaths.get(specimensScored - 1).get(1);
                    follower.followPath(scorePath, false);
                    delaySystem.CreateConditionalDelay(
                            () -> follower.getPose().getX() > specimenHangX - 0.5,
                            () -> TransferToState(AutoState.ScoringSpecimen)
                    );
                }
                break;
            case Parking:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(parkPath, true);
                    delaySystem.CreateConditionalDelay(
                            () -> !follower.isBusy(),
                            this::requestOpModeStop
                    );
                }
                break;
        }

        telemetry.addData("Specimens Scored", specimensScored);
        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

        robot.logger.log("X: " + follower.getPose().getX() + ", Y: " + follower.getPose().getY() + ", Heading: " + follower.getPose().getHeading());

        robot.Update();
        follower.update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}
