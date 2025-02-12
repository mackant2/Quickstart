package opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
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
    final Pose scorePose = new Pose(14,130, Math.toRadians(135));

    private Follower follower;

    DelaySystem delaySystem;

    boolean didStateAction = false;

    int goalSamples = 4;
    int samplesScored = 0;
    int rawStateIndex = 0;

    double intakeX = 27;

    List<Pose> samplePoses = Arrays.asList(
            new Pose(intakeX, 122.5, Math.toRadians(180)),
            new Pose(intakeX, 131.5, Math.toRadians(180)),
            new Pose(intakeX + 6, 130, Math.toRadians(223))
    );

    List<PathChain> samplePaths = new ArrayList<>();
    List<PathChain> sampleDepositPaths = new ArrayList<>();

    @Override
    public void init(){
        robot = new Robot(this, hardwareMap, false);

        delaySystem = robot.delaySystem;

        robot.arm.setClawPosition(Arm.ClawPosition.Closed);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        follower.setStartingPose(startPose);

        //add custom init path
        sampleDepositPaths.add(follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.95)
                .build());

        for (int i = 0; i < goalSamples - 1; i++) {
            Pose samplePose = samplePoses.get(i);
            Pose sampleIntakePose = new Pose(intakeX, samplePose.getY(), samplePose.getHeading());
            samplePaths.add(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePose,
                                    samplePose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose.getHeading(), samplePose.getHeading(), 0.95)
                    .build());
            sampleDepositPaths.add(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    sampleIntakePose,
                                    scorePose
                            )
                    )
                    .setLinearHeadingInterpolation(sampleIntakePose.getHeading(), scorePose.getHeading(), 0.95)
                    .build());
        }
    }
    @Override
    public void init_loop() {
        follower.update();
        robot.update();
    }


    @Override
    public void start() {
        state = SampleAutoState.MovingToBucket;
    }

    void TransferToState(SampleAutoState newState) {
        didStateAction = false;
        rawStateIndex++;
        state = newState;
    }

    @Override public void loop() {
        switch (state) {
            case MovingToBucket:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.runPreset(Arm.Presets.PRE_SAMPLE_DEPOSIT);
                    follower.followPath(sampleDepositPaths.get(samplesScored));
                }
                else if (!follower.isBusy()) {
                    TransferToState(SampleAutoState.ScoringSample);
                }
                break;
            case ScoringSample:
                if (!didStateAction) {
                    didStateAction = true;
                    delaySystem.createConditionalDelay(
                        () -> robot.arm.liftHeight >= Arm.Height.UPPER_BUCKET - 20,
                        () -> robot.arm.depositSample(() -> {
                                samplesScored++;
                                if (samplesScored < goalSamples) {
                                    TransferToState(SampleAutoState.MovingToSample);
                                }
                                else {
                                    state = SampleAutoState.Parking;
                                }
                            })
                    );
                }
                break;
            case MovingToSample:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.runPreset(Arm.Presets.RESET);
                    delaySystem.createConditionalDelay(
                            () -> robot.arm.liftHeight <= Arm.Height.PRE_TRANSFER,
                            () -> {
                                follower.followPath(samplePaths.get(samplesScored - 1), true);
                                delaySystem.createConditionalDelay(
                                        () -> !follower.isBusy(),
                                        () -> TransferToState(SampleAutoState.IntakingSample)
                                );
                            }
                    );
                }
                break;
            case IntakingSample:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.intake.runMarchingIntake(
                        samplesScored < 3 ? Intake.ExtenderPosition.OUT / 2 - 250 : Intake.ExtenderPosition.OUT / 2 - 450,
                        () -> robot.arm.transfer(() -> TransferToState(SampleAutoState.MovingToBucket)),
                        () -> {
                            if (samplesScored == goalSamples - 1) {
                                TransferToState(SampleAutoState.Parking);
                            }
                            else {
                                samplesScored++;
                                robot.intake.RunIntake(Intake.IntakeDirection.Rejecting);
                                robot.intake.extendTo(Intake.ExtenderPosition.IN + 250);
                                delaySystem.createConditionalDelay(
                                        () -> robot.intake.extenderPosition <= Intake.ExtenderPosition.IN + 250,
                                        () -> TransferToState(SampleAutoState.MovingToSample)
                                );
                            }
                        }
                    );
                }
                break;
            case Parking:
                robot.arm.runPreset(Arm.Presets.RESET);
                break;
        }

        robot.opMode.telemetry.addData("State", state);
        robot.opMode.telemetry.addData("Samples scored", samplesScored);
        telemetry.addData("Robot is moving", follower.isBusy());

        robot.logger.log("X: " + follower.getPose().getX() + ", Y: " + follower.getPose().getY() + ", Heading: " + follower.getPose().getHeading());

        follower.update();
        robot.update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}

