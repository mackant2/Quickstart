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

@Autonomous(name = "[OFFICIAL] Specimen Auto", group = "Opmodes")
public class SpecimenAuto extends OpMode {
    enum AutoState {
        Idle,
        ScoringSpecimen,
        MovingToIntake,
        IntakingSample,
        DepositingSample,
        MovingToWall,
        GoingForGrab,
        GrabbingSpecimen,
        MovingToSub,
        Parking
    }

    private AutoState state = AutoState.Idle;

    Robot robot;
    Intake intake;

    private final double robotWidth = 12.375;
    private final double robotLength = 16.125;

    private Follower follower;

    private PathChain parkPath, grabConfirmPath;

    DelaySystem delaySystem;

    boolean didStateAction = false;

    List<Pose> specimenPoses = new ArrayList<>();

    int goalSpecimens = 3;
    int specimensScored = 0;
    double sampleIntakeX = 23;
    int samplesDeposited = 0;
    double specimenHangX = 42;
    double specimenHangGap = 5;
    double startY = 72;
    double firstSampleY = 23;
    double sampleGap = 10;

    PathChain initScorePath;
    List<List<PathChain>> cyclePaths = new ArrayList<>();
    List<PathChain> intakePaths = new ArrayList<>();

    double wallX = 10.5;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        intake = robot.intake;
        delaySystem = robot.delaySystem;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        double grabY = 35;

        Pose startPose = new Pose(8.0625, startY);
        Pose grabPose = new Pose(wallX + 2, grabY);
        Pose grabConfirmPose = new Pose(wallX - 10, grabY);

        follower.setStartingPose(startPose);
        follower.holdPoint(startPose);

        grabConfirmPath = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    grabPose,
                    grabConfirmPose
                )
            )
            .setConstantHeadingInterpolation(0)
            .build();

        for (int i = 0; i < goalSpecimens; i++) {
            specimenPoses.add(new Pose(specimenHangX + 3 * i, startY + specimenHangGap * i));
        }

        initScorePath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                specimenPoses.get(0)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        for (int i = 0; i < goalSpecimens - 2; i++) {
            double startY = firstSampleY - sampleGap * (i - 1);
            Pose pathStart = i == 0 ? specimenPoses.get(0) : new Pose(sampleIntakeX, startY, Math.toRadians(180));
            Pose pathEnd = new Pose(sampleIntakeX, startY - 9, Math.toRadians(180));

            intakePaths.add(
                follower.pathBuilder()
                    .addPath(
                        i == 0 ?
                            new BezierCurve(
                                pathStart,
                                new Pose(10, 62),
                                pathEnd
                            )
                            :
                            new BezierLine(
                                pathStart,
                                pathEnd
                            )
                    )
                    .setLinearHeadingInterpolation(pathStart.getHeading(), pathEnd.getHeading(), 0.95)
                    .build()
            );
        }

        for (int i = 1; i < goalSpecimens; i++) {
            PathChain grabPath;

            if (goalSpecimens > 2) {
                Pose pathStart;
                if (i == 1) {
                    Point startPoint = intakePaths.get(intakePaths.size() - 1).getPath(0).getPoint(0);
                    pathStart = new Pose(startPoint.getX(), startPoint.getY());
                }
                else {
                    pathStart = specimenPoses.get(i - 1);
                }
                
                grabPath = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        pathStart,
                                        grabPose
                                )
                        )
                        .setConstantHeadingInterpolation(0)
                        .build();
            }
            else {
                grabPath = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        specimenPoses.get(0),
                                        //Control point that bends path to avoid getting stuck on corner of sub
                                        new Pose(10, 62),
                                        grabPose
                                )
                        )
                        .setConstantHeadingInterpolation(0)
                        .build();
            }

            PathChain scorePath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                grabConfirmPose,
                                specimenPoses.get(i)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

            cyclePaths.add(new ArrayList<>(Arrays.asList(grabPath, scorePath)));
        }

        parkPath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                specimenPoses.get(specimenPoses.size() - 1),
                                new Pose(10, 62),
                                new Pose(10, 10)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
    }

    @Override
    public void init_loop() {
        robot.update();
        follower.update();

        telemetry.addData("Specimens Scored", specimensScored);
        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
    }

    @Override
    public void start() {
        robot.arm.runPreset(Arm.Presets.PRE_SPECIMEN_DEPOSIT);
        TransferToState(AutoState.MovingToSub);
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
                    robot.arm.runPreset(Arm.Presets.SPECIMEN_DEPOSIT);
                    delaySystem.createDelay(1250, () -> {
                        robot.arm.setClawPosition(Arm.ClawPosition.OPEN);
                        specimensScored++;
                        delaySystem.createDelay(200, () -> {
                            robot.arm.runPreset(Arm.Presets.RESET);
                            if (specimensScored < goalSpecimens) {
                                if (specimensScored == 1 && goalSpecimens > 2) {
                                    TransferToState(AutoState.MovingToIntake);
                                }
                                else {
                                    TransferToState(AutoState.MovingToWall);
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
                    follower.followPath(intakePaths.get(samplesDeposited));
                    delaySystem.createConditionalDelay(
                            () -> !follower.isBusy(),
                            () -> TransferToState(AutoState.IntakingSample)
                    );
                }
                break;
            case IntakingSample:
                if (!didStateAction) {
                    didStateAction = true;
                    intake.runMarchingIntake(
                            Intake.ExtenderPosition.OUT / 2 - 150,
                            () -> robot.arm.transfer(() -> TransferToState(AutoState.DepositingSample)),
                            () -> {
                                intake.extendTo(Intake.ExtenderPosition.IN);
                                delaySystem.createConditionalDelay(
                                        intake::isExtenderIn,
                                        () -> {
                                            samplesDeposited++;
                                            if (samplesDeposited < goalSpecimens - 1) {
                                                TransferToState(AutoState.MovingToIntake);
                                            }
                                            else {
                                                TransferToState(AutoState.MovingToWall);
                                            }
                                        }
                                );
                            }
                    );
                }
                break;
            case DepositingSample:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.dropSample(() -> {
                        samplesDeposited++;
                        robot.arm.runPreset(Arm.Presets.RESET);
                        if (samplesDeposited < goalSpecimens - 2) {
                            TransferToState(AutoState.MovingToIntake);
                        }
                        else {
                            TransferToState(AutoState.MovingToWall);
                        }
                    });
                }
                break;
            case MovingToWall:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.runPreset(Arm.Presets.SPECIMEN_GRAB);
                    PathChain grabPath = cyclePaths.get(specimensScored - 1).get(0);
                    follower.followPath(grabPath);
                    delaySystem.createConditionalDelay(
                            () -> follower.getPose().getX() < wallX + 2.5,
                            () -> delaySystem.createDelay(1000, () -> TransferToState(AutoState.GoingForGrab))
                    );
                }
                break;
            case GoingForGrab:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(grabConfirmPath);
                    delaySystem.createConditionalDelay(
                            () -> follower.getPose().getX() < wallX + 1.5,
                            () -> delaySystem.createDelay(500, () -> TransferToState(AutoState.GrabbingSpecimen))
                    );
                }
            case GrabbingSpecimen:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.setClawPosition(Arm.ClawPosition.CLOSED);
                    delaySystem.createDelay(200, () -> TransferToState(AutoState.MovingToSub));
                }
                break;
            case MovingToSub:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.runPreset(Arm.Presets.PRE_SPECIMEN_DEPOSIT);
                    PathChain scorePath = specimensScored == 0 ? initScorePath : cyclePaths.get(specimensScored - 1).get(1);
                    follower.followPath(scorePath, false);
                    delaySystem.createConditionalDelay(
                            () -> follower.getPose().getX() > specimenHangX - 5.5,
                            () -> TransferToState(AutoState.ScoringSpecimen)
                    );
                }
                break;
            case Parking:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(parkPath, true);
                    delaySystem.createConditionalDelay(
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

        robot.update();
        follower.update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}
