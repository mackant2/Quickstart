package components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.DelaySystem;
import utils.PressEventSystem;
import utils.Robot;

public class Arm {
    public enum State {
        DriverControlled,
        Transferring
    }
    public static class FourBarPosition {
        public static final double STRAIGHT_BACK = 0.21;
        public static final double STRAIGHT_UP = 0.50;
        public static final double SAMPLE_DEPOSIT = 0.68;
    }
    public static class WristPosition {
        public static final double TRANSFER = .86;
        public static final double STRAIGHT = 0.5;
        public static final double SPECIMEN_DEPOSIT = 0.15;
        public static final double SAMPLE_DEPOSIT = 0.3;
    }
    public static class Height {
        public static final int UPPER_BUCKET = 3400;
        public static final int DOWN = 0;
        public static final int TRANSFER = 100;
        public static final int PRE_TRANSFER = 200;
        public static int WALL_PICKUP = 157;
        public static final int PRE_SPECIMEN_DEPOSIT = 920;
        public static final int SPECIMEN_DEPOSIT = 1600;
    }
    public static class ClawPosition {
        public static final double Open = 0.9;
        public static final double Closed = 0;
    }
    public State state = State.DriverControlled;
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

    public void DepositSample() {
        RotateFourBar(FourBarPosition.SAMPLE_DEPOSIT);
        RotateWrist(WristPosition.SAMPLE_DEPOSIT);
        delaySystem.CreateDelay(500, () -> {
            robot.arm.SetClawPosition(Arm.ClawPosition.Open);
        });
    }

    public void DepositSample(Runnable callback) {
        RotateFourBar(FourBarPosition.SAMPLE_DEPOSIT);
        RotateWrist(WristPosition.SAMPLE_DEPOSIT);
        delaySystem.CreateDelay(500, () -> {
            robot.arm.SetClawPosition(Arm.ClawPosition.Open);
            delaySystem.CreateDelay(500, callback::run);
        });
    }

    public void UpdateWallPickupHeight() {
        robot.logger.log("WALL PICKUP HEIGHT: " + liftLeft.getTargetPosition());
        Height.WALL_PICKUP = liftLeft.getTargetPosition();
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

    public void RegisterControls() {
        PressEventSystem pressEventSystem = robot.pressEventSystem;

        pressEventSystem.AddListener(assistantController, "left_bumper", robot.arm::UpdateWallPickupHeight);
        pressEventSystem.AddListener(assistantController, "right_bumper", robot.arm::ToggleClaw);
        pressEventSystem.AddListener(assistantController, "y", () -> robot.arm.RunPreset(Arm.Presets.PRE_SAMPLE_DEPOSIT));
        pressEventSystem.AddListener(assistantController, "a", () -> robot.arm.RunPreset(Presets.SPECIMEN_GRAB));
        pressEventSystem.AddListener(assistantController, "x", () -> robot.arm.RunPreset(Arm.Presets.PRE_SPECIMEN_DEPOSIT));
        pressEventSystem.AddListener(assistantController, "dpad_down", () -> robot.arm.RunPreset(Arm.Presets.PRE_TRANSFER));
        pressEventSystem.AddListener(assistantController, "dpad_left", robot.arm::Transfer);
        pressEventSystem.AddListener(assistantController, "dpad_up", robot.arm::DepositSample);
    }

    public void Initialize() {
        RunPreset(Presets.RESET);
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
            if (state != State.DriverControlled) {
                state = State.DriverControlled;
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
        if (change != 0 && state != State.DriverControlled) {
            state = State.DriverControlled;
        }
        //Math.clamp causes crash here, so using custom method
        double clamped = clamp((float)(fourBarPosition + change * 0.5), (float)FourBarPosition.STRAIGHT_BACK, 1);
        if (change != 0) {
            if (state != State.DriverControlled) {
                state = State.DriverControlled;
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

        robot.logger.log("Lift Position: " + GetLiftHeight() + ", Lift Velocity: " + liftLeft.getVelocity() + ", Lift Voltage (MILLIAMPS): " + liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public int GetLiftHeight() {
        return liftLeft.getCurrentPosition();
    }

    public void Transfer() {
        if (state != State.Transferring) {
            state = State.Transferring;
            RunPreset(Arm.Presets.PRE_TRANSFER);
            robot.intake.ExtendTo(Intake.ExtenderPosition.IN);
            delaySystem.CreateConditionalDelay(
                    robot.intake::isExtenderIn,
                    () -> {
                        RunPreset(Arm.Presets.TRANSFER);
                        delaySystem.CreateDelay(500, () -> {
                            SetClawPosition(Arm.ClawPosition.Closed);
                            state = State.DriverControlled;
                        });
                    }
            );
        }
    }
    
    public void Transfer(Runnable callback) {
        if (state != State.Transferring) {
            state = State.Transferring;
            RunPreset(Presets.PRE_TRANSFER);
            robot.intake.ExtendTo(Intake.ExtenderPosition.IN);
            delaySystem.CreateConditionalDelay(
                    robot.intake::isExtenderIn,
                    () -> {
                        RunPreset(Arm.Presets.TRANSFER);
                        delaySystem.CreateConditionalDelay(
                            () -> GetLiftHeight() <= Height.TRANSFER,
                            () -> {
                                SetClawPosition(Arm.ClawPosition.Closed);
                                state = State.DriverControlled;
                                delaySystem.CreateDelay(250, callback::run);
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
        public static final Preset RESET = new Preset(Height.DOWN, FourBarPosition.STRAIGHT_UP, 0.1, ClawPosition.Closed);
        public static final Preset PRE_TRANSFER = new Preset(Height.PRE_TRANSFER, FourBarPosition.STRAIGHT_BACK, WristPosition.TRANSFER, ClawPosition.Open);
        public static final Preset TRANSFER = new Preset(Height.TRANSFER, FourBarPosition.STRAIGHT_BACK, WristPosition.TRANSFER, ClawPosition.Open);
        public static final Preset PRE_SAMPLE_DEPOSIT = new Preset(Height.UPPER_BUCKET, FourBarPosition.STRAIGHT_UP, WristPosition.STRAIGHT, ClawPosition.Closed);
        public static final Preset SPECIMEN_GRAB = new Preset(Height.WALL_PICKUP, FourBarPosition.STRAIGHT_BACK, WristPosition.STRAIGHT, ClawPosition.Open);
        public static final Preset PRE_SPECIMEN_DEPOSIT = new Preset(Height.PRE_SPECIMEN_DEPOSIT, FourBarPosition.STRAIGHT_UP, WristPosition.SPECIMEN_DEPOSIT, ClawPosition.Closed);
        public static final Preset SPECIMEN_DEPOSIT = new Preset(Height.SPECIMEN_DEPOSIT, FourBarPosition.STRAIGHT_UP, WristPosition.SPECIMEN_DEPOSIT, ClawPosition.Closed);
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