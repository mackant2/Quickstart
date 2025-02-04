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
        Intaking
    }
    private SampleAuto.SampleAutoState state = SampleAuto.SampleAutoState.Idle;

    Robot robot;

    private final double robotWidth = 12.3;
    private final double robotLength = 15.75;

    private Follower follower;

    private PathChain initPath;

    DelaySystem delaySystem;

    boolean didStateAction = false;

    int goalSamples = 1;
    int samplesScored = 0;
    @Override
    public void init(){
        robot = new Robot(this, hardwareMap, false);

        delaySystem = robot.delaySystem;

        robot.arm.SetClawPosition(Arm.ClawPosition.Closed);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        Pose startPose = new Pose(7.875, 113.85, Math.PI / 2);
        Pose initScorePose = new Pose(7.875,125, Math.PI / 2);

        follower.setStartingPose(startPose);

        initPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(initScorePose)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), initScorePose.getHeading())
                .build();
    }
    @Override
    public void init_loop() {
        follower.update();
        robot.Update();
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
                    follower.followPath(initPath, true);
                    delaySystem.CreateDelay(1500, () -> {
                        robot.arm.SetClawPosition(Arm.ClawPosition.Open);
                        samplesScored++;
                        if (samplesScored < goalSamples) {
                            didStateAction = false;
                            state = SampleAutoState.Intaking;
                        }
                        else {
                            requestOpModeStop();
                        }
                    });
                }
                break;
        }

        robot.opMode.telemetry.addData("State", state);

        robot.logger.Log("X: " + follower.getPose().getX());
        robot.logger.Log("Y: " + follower.getPose().getY());

        follower.update();
        robot.Update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}

