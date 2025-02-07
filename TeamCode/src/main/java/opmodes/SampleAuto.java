package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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

@Autonomous(name = "Bucket Auto", group = "Opmodes")
public class SampleAuto extends OpMode {
    enum SampleAutoState {
        Idle,
        MovingToBucket,
        ScoringSample,
        MovingToSample,
        IntakingSample,
        Parking
    }
    private SampleAutoState state = SampleAutoState.Idle;

    Robot robot;

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private final Pose startPose = new Pose(7.875, 113.85, Math.toRadians(90));
    final Pose scorePose = new Pose(11.5,123.2, Math.toRadians(135));

    private Follower follower;

    DelaySystem delaySystem;

    boolean didStateAction = false;

    int goalSamples = 3;
    int samplesScored = 0;

    List<Pose> samplePoses = Arrays.asList(
            new Pose(11.5, 123.1, Math.toRadians(155)),
            new Pose(14.5, 123.2, Math.toRadians(176))
    );

    List<PathChain> samplePaths = new ArrayList<>();
    List<PathChain> sampleIntakePaths = new ArrayList<>();
    List<PathChain> sampleDepositPaths = new ArrayList<>();

    @Override
    public void init(){
        robot = new Robot(this, hardwareMap, false);

        delaySystem = robot.delaySystem;

        robot.arm.SetClawPosition(Arm.ClawPosition.Closed);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        follower.setStartingPose(startPose);

        //add custom init path
        sampleDepositPaths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build());

        for (int i = 0; i < goalSamples - 1; i++) {
            Pose samplePose = samplePoses.get(i);
            Pose sampleIntakePose = new Pose(samplePose.getX() + 6, samplePose.getY(), samplePose.getHeading());
            samplePaths.add(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(scorePose),
                                    new Point(samplePose)
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose.getHeading(), samplePose.getHeading())
                    .build());
            sampleIntakePaths.add(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(samplePose),
                                    new Point(sampleIntakePose)
                            )
                    )
                    .setLinearHeadingInterpolation(samplePose.getHeading(), sampleIntakePose.getHeading())
                    .build());
            sampleDepositPaths.add(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(sampleIntakePose),
                                    new Point(scorePose)
                            )
                    )
                    .setLinearHeadingInterpolation(sampleIntakePose.getHeading(), scorePose.getHeading())
                    .build());
        }
    }
    @Override
    public void init_loop() {
        follower.update();
        robot.Update();
    }


    @Override
    public void start() {
        state = SampleAutoState.MovingToBucket;
    }

    void TransferToState(SampleAutoState newState) {
        didStateAction = false;
        state = newState;
    }

    @Override public void loop() {
        switch (state) {
            case MovingToBucket:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.RotateWrist(Arm.WristPosition.STRAIGHT);
                    robot.arm.GoToHeight(Arm.Height.UPPER_BUCKET);
                    follower.followPath(sampleDepositPaths.get(samplesScored), true);
                }
                else if (!follower.isBusy()) {
                    TransferToState(SampleAutoState.ScoringSample);
                }
                break;
            case ScoringSample:
                if (!didStateAction) {
                    didStateAction = true;
                    delaySystem.CreateConditionalDelay(
                        () -> robot.arm.GetLiftHeight() >= Arm.Height.UPPER_BUCKET,
                        () -> {
                            robot.arm.DepositSample(() -> {
                                samplesScored++;
                                if (samplesScored < goalSamples) {
                                    TransferToState(SampleAutoState.MovingToSample);
                                }
                                else {
                                    state = SampleAutoState.Parking;
                                }
                            });
                        }
                    );
                }
                break;
            case MovingToSample:
                if (!didStateAction) {
                    didStateAction = true;
                    follower.followPath(samplePaths.get(samplesScored - 1), true);
                    delaySystem.CreateConditionalDelay(
                            () -> !follower.isBusy(),
                            () -> {
                                robot.intake.ExtendTo(Intake.ExtenderPosition.OUT);
                                delaySystem.CreateConditionalDelay(
                                        () -> robot.intake.GetExtenderPosition() >= Intake.ExtenderPosition.OUT,
                                        () -> TransferToState(SampleAutoState.IntakingSample)
                                );
                            }
                    );
                }
                break;
            case IntakingSample:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.RunPreset(Arm.Presets.PRETRANSFER);
                    robot.intake.SetFlipdownPosition(Intake.FlipdownPosition.DOWN);
                    robot.intake.RunIntake(Intake.IntakeDirection.Rejecting);
                    follower.followPath(sampleIntakePaths.get(samplesScored - 1), true);
                    delaySystem.CreateConditionalDelay(
                        () -> follower.getPose().getX() >= 15,
                        () -> {
                            robot.intake.RunIntake(Intake.IntakeDirection.Intaking);
                            delaySystem.CreateConditionalDelay(
                                robot.parsedHardwareMap.intakeLimiter::isPressed,
                                () -> {
                                    robot.intake.StopIntake();
                                    robot.arm.Transfer(() -> {
                                        TransferToState(SampleAutoState.MovingToBucket);
                                    });
                                }
                            );
                            delaySystem.CreateDelay(
                                    3000,
                                    () -> {
                                        if (state == SampleAutoState.IntakingSample && robot.arm.state != Arm.ArmState.Transferring) {
                                            TransferToState(SampleAutoState.Parking);
                                        }
                                    }
                            );
                        }
                    );
                }
                break;
            case Parking:
                requestOpModeStop();
                break;
        }

        robot.opMode.telemetry.addData("State", state);
        robot.opMode.telemetry.addData("Samples scored", samplesScored);
        telemetry.addData("Robot is moving", follower.isBusy());

        robot.logger.Log("X: " + follower.getPose().getX() + ", Y: " + follower.getPose().getY() + ", Heading: " + follower.getPose().getHeading());

        follower.update();
        robot.Update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}

