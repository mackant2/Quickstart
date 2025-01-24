package components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.Robot;

public class Arm {
    public enum ArmState {
        DriverControlled,
        Transferring
    }
    public static class FourBarPosition {
        public static final double Transfer = 0.09;
        public static final double Extraction = 0;
        public static final double Specimen = 0.81;
    }
    public static class WristPosition {
        public static final double Transfer = .56;
        public static final double Specimen = 0.4;//.58
        public static final double Straight = 0.5;
        public static final double SampleDrop = 0.53;
    }
    public static class Height {
        public static final int LOWER_BUCKET = 2400;
        public static final int UPPER_BUCKET = 3700;
        public static final int UPPER_BAR = 2000;
        public static final int DOWN = 0;
        public static final int Transfer = 300;
        public static final int ExtractionComplete = 480;
        public static int WallPickup = 230;
    }
    public static class ClawPosition {
        public static final double Open = 1;
        public static final double Closed = 0;
    }
    Telemetry telemetry;
    Gamepad assistantController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    final double MAX_FOURBAR_SPEED = 0.05;
    final float MAX_HEIGHT = Height.UPPER_BUCKET;
    final int LIFT_MAX_DIFF = 400;
    public StateMachine stateMachine;
    Robot robot;

    public void RotateFourBar(double position) {
        leftFourBar.setPosition(position);
        rightFourBar.setPosition(position);
    }

    public void RotateWrist(double degrees) {
        wrist.setPosition(degrees);
        //wrist.setPosition(degrees / 180f + 0.5f);
    }

    double GetWristDegrees() {
        return wrist.getPosition() / 180 + 0.5;
    }

    public void PrepareToGrabSpecimen() {
        RotateFourBar(FourBarPosition.Specimen);
        wrist.setPosition(WristPosition.Straight);
        claw.setPosition(ClawPosition.Open);
        GoToHeight(Height.WallPickup);
    }

    public void PrepareToDepositSpecimen() {
        GoToHeight(Height.UPPER_BAR);
        RotateFourBar(FourBarPosition.Specimen);
        wrist.setPosition(WristPosition.Specimen);
    }

    public void PrepareToTransfer() {
        GoToHeight(Height.DOWN);
        RotateFourBar(FourBarPosition.Transfer);
        RotateWrist(WristPosition.Transfer);
    }

    public void UpdateWallPickupHeight() {
        robot.logger.Log("WALL PICKUP HEIGHT: " + liftLeft.getTargetPosition());
        Height.WallPickup = liftLeft.getTargetPosition();
    }

    public Arm(Robot robot) {
        this.robot = robot;
        this.assistantController = robot.opMode.gamepad2;
        this.telemetry = robot.opMode.telemetry;

        //assign motors
        liftLeft = robot.parsedHardwareMap.liftLeft;
        liftRight = robot.parsedHardwareMap.liftRight;
        //assign servos
        //four bar - 0 is out, 1 is transfer position
        leftFourBar = robot.parsedHardwareMap.leftFourBar;
        rightFourBar = robot.parsedHardwareMap.rightFourBar;
        wrist = robot.parsedHardwareMap.wrist;
        claw = robot.parsedHardwareMap.claw;

        stateMachine = new StateMachineBuilder()
            .state(ArmState.DriverControlled)
            .transition(() -> stateMachine.getState() == ArmState.Transferring)
            .state(ArmState.Transferring)
            .transition(() -> leftFourBar.getPosition() == 0.5, ArmState.DriverControlled)
            .build();
    }

    public void Initialize() {
        stateMachine.start();
        GoToHeight(Height.Transfer);
        RotateFourBar(FourBarPosition.Transfer);
        wrist.setPosition(WristPosition.Transfer);
        claw.setPosition(ClawPosition.Open);
    }

    public void ToggleClaw() {
        if (stateMachine.getState() == ArmState.DriverControlled) {
            claw.setPosition(claw.getPosition() == ClawPosition.Open ? ClawPosition.Closed : ClawPosition.Open);
        }
    }

    public void SetClawPosition(double newPosition) {
        claw.setPosition(newPosition);
    }

    public void GoToHeight(int height) {
        liftLeft.setTargetPosition(height);
        liftRight.setTargetPosition(height);
    }

    public void AdjustLiftHeight(int change) {
        GoToHeight(liftLeft.getCurrentPosition() + change);
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    boolean transferInitiated = false;
    boolean extractingEntered = false;

    public void Update() {
        stateMachine.update();
        ArmState state = (ArmState)stateMachine.getState();

        switch (state) {
            case DriverControlled:
                double power = -assistantController.left_stick_y;
                if (power != 0) {
                    if (assistantController.left_bumper) {
                        power *= 0.2;
                    }
                    GoToHeight((int)clamp((float)(liftLeft.getCurrentPosition() + 1 + Math.floor(power * LIFT_MAX_DIFF)), 0, MAX_HEIGHT));
                }

                double leftPos = leftFourBar.getPosition();
                double change = -assistantController.right_stick_y * MAX_FOURBAR_SPEED;
                if (assistantController.right_bumper) {
                    change *= 0.5;
                }
                //Math.clamp causes crash here, so using custom method
                double leftClamped = clamp((float)(leftPos + change), (float)FourBarPosition.Transfer, 1);
                if (change != 0) {
                    RotateFourBar(leftClamped);
                }

                wrist.setPosition(clamp((float)(wrist.getPosition() + (assistantController.left_trigger - assistantController.right_trigger) * 0.02), 0, 1));
            break;
            case Transferring:
                if (!extractingEntered) {
                    //reset state
                    extractingEntered = true;
                    claw.setPosition(ClawPosition.Closed);
                    liftLeft.setVelocity(500);
                    GoToHeight(Height.ExtractionComplete);
                }
                int liftDistanceFromTransfer = liftLeft.getCurrentPosition() - Height.Transfer;
                int liftDistanceFromExtraction = Height.ExtractionComplete - Height.Transfer;
                double completionPercentage = clamp((float)((double)liftDistanceFromTransfer / liftDistanceFromExtraction), 0, 1);
                //Move both four bar and wrist to 0 throughout sample extraction from transfer plate
                double fourBarDiff = FourBarPosition.Extraction - FourBarPosition.Transfer;
                RotateFourBar(FourBarPosition.Transfer + completionPercentage * fourBarDiff);
                if (completionPercentage >= 1) {
                    extractingEntered = false;
                    RotateFourBar(0.5);
                    wrist.setPosition(0);
                    liftLeft.setVelocity(1000);
                }
            break;
        }

        telemetry.addData("Arm State", state);
        robot.opMode.telemetry.addData("Arm Voltage (MILLIAMPS)", liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Lift Position", liftLeft.getCurrentPosition());
        telemetry.addData("Lift Target", liftLeft.getTargetPosition());
        telemetry.addData("Four Bar Position", leftFourBar.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
    }

    public void GoToSpecimenPosition() {
        claw.setPosition(ClawPosition.Closed);
        RotateFourBar(FourBarPosition.Specimen);
        wrist.setPosition(WristPosition.Specimen);
        GoToHeight(Height.UPPER_BAR);
    }

    public void HangSpecimen() {
        GoToHeight(1300);
    }

    public void ReleaseSpecimen() {
        claw.setPosition(ClawPosition.Open);
        wrist.setPosition(0.5);
    }

    public void Reset() {
        GoToHeight(Height.DOWN);
        wrist.setPosition(WristPosition.Straight);
        RotateFourBar(.5);
        claw.setPosition(ClawPosition.Closed);
    }
}