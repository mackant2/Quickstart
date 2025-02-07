package components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import utils.DelaySystem;
import utils.Robot;

public class Arm {
    public enum ArmState {
        DriverControlled,
        Transferring
    }
    public static class FourBarPosition {
        public static final double Bucket = .68;
        public static final double SpecimenHang = 1;
        public static final double StraightBack = 0.21;
        public static final double StraightUp = 0.50;
        public static final double StraightOut = .8;
        public static final double SampleDeposit = 0.68;
    }
    public static class WristPosition {
        public static final double Transfer = .86;
        public static final double Specimen = 0.69;//.58
        public static final double Straight = 0.5;
        public static final double SPECIMEN_DEPOSIT = .35;
        public static final double SampleDeposit = 0.3;
    }
    public static class Height {
        public static final int LOWER_BUCKET = 2400;
        public static final int UPPER_BUCKET = 3650;
        public static final int UPPER_BAR = 1800;
        public static final int DOWN = 0;
        public static final int Transfer = 100;
        public static int WallPickup = 157;
        public static final int SPECIMEN_DEPOSIT = 650;
    }
    public static class ClawPosition {
        public static final double Open = 0.9;
        public static final double Closed = 0;
    }
    public ArmState state = ArmState.DriverControlled;
    Telemetry telemetry;
    Gamepad assistantController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    final double MAX_FOURBAR_SPEED = 0.05;
    final float MAX_HEIGHT = Height.UPPER_BUCKET;
    final int LIFT_MAX_DIFF = 400;
    Robot robot;
    public DelaySystem delaySystem;

    public void RotateFourBar(double position) {
        leftFourBar.setPosition(1 - position);
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
        RotateFourBar(FourBarPosition.StraightBack);
        wrist.setPosition(WristPosition.Straight);
        //claw.setPosition(ClawPosition.Open);
        claw.setPosition(ClawPosition.Open); //TODO: go back to open claw position after new claw is printed and fourbar passthrough with open claw is allowed
        GoToHeight(Height.WallPickup);
    }

    public void DepositSample() {
        RotateFourBar(FourBarPosition.SampleDeposit);
        RotateWrist(WristPosition.SampleDeposit);
        delaySystem.CreateDelay(1000, () -> {
            robot.arm.SetClawPosition(Arm.ClawPosition.Open);
        });
    }

    public void DepositSample(Runnable callback) {
        RotateFourBar(FourBarPosition.SampleDeposit);
        RotateWrist(WristPosition.SampleDeposit);
        delaySystem.CreateDelay(1000, () -> {
            robot.arm.SetClawPosition(Arm.ClawPosition.Open);
            delaySystem.CreateDelay(500, callback::run);
        });
    }

    public void UpdateWallPickupHeight() {
        robot.logger.Log("WALL PICKUP HEIGHT: " + liftLeft.getTargetPosition());
        Height.WallPickup = liftLeft.getTargetPosition();
    }

    public Arm(Robot robot) {
        this.robot = robot;
        this.assistantController = robot.opMode.gamepad2;
        this.telemetry = robot.opMode.telemetry;
        this.delaySystem = robot.delaySystem;

        //assign motors
        liftLeft = robot.parsedHardwareMap.liftLeft;
        liftRight = robot.parsedHardwareMap.liftRight;
        //assign servos
        //four bar - 0 is out, 1 is transfer position
        leftFourBar = robot.parsedHardwareMap.leftFourBar;
        rightFourBar = robot.parsedHardwareMap.rightFourBar;
        wrist = robot.parsedHardwareMap.wrist;
        claw = robot.parsedHardwareMap.claw;
    }

    public void Initialize() {
        SetClawPosition(Arm.ClawPosition.Closed);
        delaySystem.CreateDelay(1500, () -> RunPreset(Presets.RESET));
    }

    public void ToggleClaw() {
        claw.setPosition(claw.getPosition() == ClawPosition.Open ? ClawPosition.Closed : ClawPosition.Open);
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

    public void Update() {
        double power = -assistantController.left_stick_y;
        if (power != 0) {
            if (state != ArmState.DriverControlled) {
                state = ArmState.DriverControlled;
            }
            if (assistantController.left_bumper) {
                power *= 0.2;
            }
            GoToHeight((int)clamp((float)(liftLeft.getCurrentPosition() + 1 + Math.floor(power * LIFT_MAX_DIFF)), 0, MAX_HEIGHT));
        }

        double fourBarPosition = rightFourBar.getPosition();
        double change = -assistantController.right_stick_y * MAX_FOURBAR_SPEED;
        if (assistantController.right_bumper) {
            change *= 0.5;
        }
        if (change != 0 && state != ArmState.DriverControlled) {
            state = ArmState.DriverControlled;
        }
        //Math.clamp causes crash here, so using custom method
        double clamped = clamp((float)(fourBarPosition + change * 0.5), (float)FourBarPosition.StraightBack, 1);
        if (change != 0) {
            if (state != ArmState.DriverControlled) {
                state = ArmState.DriverControlled;
            }
            RotateFourBar(clamped);
        }

        wrist.setPosition(clamp((float)(wrist.getPosition() + (assistantController.left_trigger - assistantController.right_trigger) * 0.02), 0, 1));
        telemetry.addData("Arm State", state);
        robot.opMode.telemetry.addData("Arm Voltage (MILLIAMPS)", liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Lift Position", liftLeft.getCurrentPosition());
        telemetry.addData("Lift Target", liftLeft.getTargetPosition());
        telemetry.addData("Four Bar Position", leftFourBar.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
    }

    public void HangSpecimen() {
        GoToHeight(2700);
    }

    public int GetLiftHeight() {
        return liftLeft.getCurrentPosition();
    }

    public void Transfer() {
        if (state != ArmState.Transferring) {
            state = Arm.ArmState.Transferring;
            RunPreset(Arm.Presets.PRETRANSFER);
            robot.intake.ExtendTo(Intake.ExtenderPosition.IN);
            delaySystem.CreateConditionalDelay(
                    () -> robot.intake.GetExtenderPosition() <= Intake.ExtenderPosition.IN,
                    () -> {
                        RunPreset(Arm.Presets.TRANSFER);
                        delaySystem.CreateDelay(500, () -> {
                            SetClawPosition(Arm.ClawPosition.Closed);
                            state = ArmState.DriverControlled;
                        });
                    }
            );
        }
    }
    
    public void Transfer(Runnable callback) {
        if (state != ArmState.Transferring) {
            state = Arm.ArmState.Transferring;
            RunPreset(Arm.Presets.PRETRANSFER);
            robot.intake.ExtendTo(Intake.ExtenderPosition.IN);
            delaySystem.CreateConditionalDelay(
                    () -> robot.intake.GetExtenderPosition() <= Intake.ExtenderPosition.IN,
                    () -> {
                        RunPreset(Arm.Presets.TRANSFER);
                        delaySystem.CreateConditionalDelay(
                            () -> GetLiftHeight() <= Height.Transfer,
                            () -> {
                                SetClawPosition(Arm.ClawPosition.Closed);
                                state = ArmState.DriverControlled;
                                delaySystem.CreateDelay(500, callback::run);
                        });
                    }
            );
        }
    }

    public void RunPreset(Preset preset) {
        GoToHeight(preset.liftPosition);
        RotateFourBar(preset.fourBarPosition);
        RotateWrist(preset.wristPosition);
        SetClawPosition(preset.clawPosition);
    }

    public static class Presets {
        public static final Preset RESET = new Preset(Height.DOWN, FourBarPosition.StraightUp, 0.1, ClawPosition.Closed);
        public static final Preset PRETRANSFER = new Preset(Height.Transfer + 200, FourBarPosition.StraightBack, WristPosition.Transfer, ClawPosition.Open);
        public static final Preset TRANSFER = new Preset(Height.Transfer, FourBarPosition.StraightBack, WristPosition.Transfer, ClawPosition.Open);
        public static final Preset PRE_SAMPLE_DEPOSIT = new Preset(Height.UPPER_BUCKET, FourBarPosition.StraightUp, WristPosition.Straight, ClawPosition.Closed);
        public static final Preset PRE_SPECIMEN_DEPOSIT = new Preset(Height.UPPER_BAR, FourBarPosition.StraightOut, WristPosition.Specimen, ClawPosition.Closed);
        public static final Preset SPECIMEN_DEPOSIT = new Preset(Height.SPECIMEN_DEPOSIT, FourBarPosition.StraightUp, WristPosition.SPECIMEN_DEPOSIT, ClawPosition.Closed);
    }

    public static class Preset {
        int liftPosition;
        double fourBarPosition;
        double wristPosition;
        double clawPosition;

        public Preset(int liftPosition, double fourBarPosition, double wristPosition, double clawPosition) {
            this.liftPosition = liftPosition;
            this.fourBarPosition = fourBarPosition;
            this.wristPosition = wristPosition;
            this.clawPosition = clawPosition;
        }
    }
}