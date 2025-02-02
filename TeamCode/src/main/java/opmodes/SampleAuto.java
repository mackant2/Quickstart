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
import java.util.List;

import components.Arm;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import utils.DelaySystem;
import utils.Robot;

@Autonomous(name = "Bucket Auto", group = "Opmodes")
public class SampleAuto extends OpMode {
    enum SampleAutoState {
        Idle,
        ScoringSample,
        Intaking,
        DoingScoreMove,
        Parking
    }
    private SampleAuto.SampleAutoState state = SampleAuto.SampleAutoState.Idle;

    Robot robot;

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private Follower follower;

    private PathChain score1Path, intake1Path;

    DelaySystem delaySystem = new DelaySystem();

    boolean didStateAction = false;

    int goalSample = 1;
    int sampleScored = 0;
    @Override
    public void init(){
        robot = new Robot(this, hardwareMap, false);

        robot.arm.SetClawPosition(Arm.ClawPosition.Closed);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(38.88, 64.23, -Math.PI / 2);
        Pose score1Pose = new Pose(6.15,125, -Math.PI / 2);
        Pose intake1Pose = new Pose(18,125);

        follower.setStartingPose(startPose);

        score1Path = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(score1Pose)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        intake1Path = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score1Pose),
                                new Point(intake1Pose)
                        )
                )
                .setLinearHeadingInterpolation(score1Pose.getHeading(), intake1Pose.getHeading())
                .build();

    }
    @Override
    public void init_loop() {
        follower.update();
        delaySystem.Update();
    }


    @Override
    public void start() {
        state = SampleAutoState.ScoringSample;
    }

    @Override public void loop() {
        switch (state) {
            case ScoringSample:
                if (!didStateAction) {
                    didStateAction = true;
                    robot.arm.ScoreSample();
                    follower.followPath(score1Path);
                    delaySystem.CreateDelay(1500, () -> {
                        robot.arm.SetClawPosition(Arm.ClawPosition.Open);
                        sampleScored++;
                        if (sampleScored < goalSample) {
                            state = SampleAuto.SampleAutoState.Intaking;
                        }
                        /*else {
                            state = Auto.AutoState.Parking;
                        }*/
                    });
                }
                break;
            case Intaking:
                follower.followPath(intake1Path);
                break;
            /*case DoingScoreMove:
                break;
            case Parking:
                break;*/
        }

        follower.update();
        delaySystem.Update();
    }

    @Override
    public void stop() {

    }
}

