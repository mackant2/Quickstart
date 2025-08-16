package components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import utils.DelaySystem;
import utils.PressEventSystem;
import utils.Robot;

public class Arm {
    public enum State {
        DriverControlled,
        Transferring,
        Hanging
    }
    public static class FourBarPosition {
        public static final double STRAIGHT_FORWARD = 0.83;
        public static final double STRAIGHT_BACK = 0.21;
        public static final double STRAIGHT_UP = 0.50;
        public static final double SAMPLE_DEPOSIT = 0.63;
        public static final double SPECIMEN_DEPOSIT = 0.53;
    }
    public static class WristPosition {
        public static final double TRANSFER = .86;
        public static final double STRAIGHT = 0.5;
        public static final double SPECIMEN_DEPOSIT = 0.31;
        public static final double SAMPLE_DEPOSIT = 0.37;
    }
    public static class Height {
        public static final int UPPER_BUCKET = 3200;
        public static final int DOWN = 0;
        public static final int TRANSFER = 50;
        public static final int PRE_TRANSFER = 200;
        public static int WALL_PICKUP = 160;
        public static final int PRE_SPECIMEN_DEPOSIT = 750;
        public static final int SPECIMEN_DEPOSIT = 1400;
    }
    public static class ClawPosition {
        public static final double OPEN = 0.9;
        public static final double CLOSED = 0;
    }
    public State state = State.DriverControlled;
    Telemetry telemetry;
    Gamepad assistantController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    final double MAX_FOUR_BAR_SPEED = 0.05;
    final float MAX_HEIGHT = Height.UPPER_BUCKET;
    final int LIFT_MAX_DIFF = 400;
    Robot robot;
    public DelaySystem delaySystem;
    public int liftHeight = 0;
    int liftTarget = 0;
    double wristPosition = 0;
    boolean hangPrimed = false;

    public void rotateFourBar(double position) {
        leftFourBar.setPosition(1 - position);
        rightFourBar.setPosition(position);
    }

    public void rotateWrist(double position) {
        wristPosition = position;
        wrist.setPosition(wristPosition);
    }

    public void depositSample() {
        rotateFourBar(FourBarPosition.SAMPLE_DEPOSIT);
        rotateWrist(WristPosition.SAMPLE_DEPOSIT);
        delaySystem.createDelay(300, this::openClaw);
    }

    public void depositSample(Runnable callback) {
        rotateFourBar(FourBarPosition.SAMPLE_DEPOSIT);
        rotateWrist(WristPosition.SAMPLE_DEPOSIT);
        delaySystem.createDelay(300, () -> openClaw(callback));
    }

    public void updateWallPickupHeight() {
        Height.WALL_PICKUP = liftHeight;
        robot.logger.log("WALL PICKUP HEIGHT: " + liftTarget);
    }

    public void openClaw() {
        setClawPosition(ClawPosition.OPEN);
    }

    public void openClaw(Runnable callback) {
        openClaw();
        delaySystem.createDelay(200, callback);
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

    public void registerControls() {
        PressEventSystem pressEventSystem = robot.pressEventSystem;

        pressEventSystem.addListener(assistantController, "left_bumper", this::updateWallPickupHeight);
        pressEventSystem.addListener(assistantController, "right_bumper", this::toggleClaw);
        pressEventSystem.addListener(assistantController, "y", () -> runPreset(Presets.PRE_SAMPLE_DEPOSIT));
        pressEventSystem.addListener(assistantController, "a", () -> runPreset(Presets.SPECIMEN_GRAB));
        pressEventSystem.addListener(assistantController, "x", () -> {
            setClawPosition(ClawPosition.CLOSED);
            delaySystem.createDelay(200, () -> runPreset(Presets.PRE_SPECIMEN_DEPOSIT));
        });
        pressEventSystem.addListener(assistantController, "b", this::primeHang);
        pressEventSystem.addListener(assistantController, "dpad_down", () -> runPreset(Presets.PRE_TRANSFER));
        pressEventSystem.addListener(assistantController, "dpad_left", this::transfer);
        pressEventSystem.addListener(assistantController, "dpad_up", this::depositSample);
        pressEventSystem.addListener(assistantController, "dpad_right", this::hang);
    }

    public void toggleClaw() {
        setClawPosition(claw.getPosition() == ClawPosition.OPEN ? ClawPosition.CLOSED : ClawPosition.OPEN);
    }

    public void setClawPosition(double newPosition) {
        claw.setPosition(newPosition);
    }

    public void goToHeight(int height) {
        if (liftTarget == height) return;

        liftTarget = height;
        liftLeft.setTargetPosition(liftTarget);
        liftRight.setTargetPosition(liftTarget);
    }

    public void adjustLiftHeight(int change) {
        goToHeight(liftHeight + change);
    }

    public void primeHang() {
        hangPrimed = !hangPrimed;
    }

    public void hang() {
        if (hangPrimed) {
            state = State.Hanging;
            goToHeight(liftHeight - 1100);
        }
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void update() {
        double wristPosition = wrist.getPosition();

        double power = -assistantController.left_stick_y;
        if (power != 0) {
            if (state != State.DriverControlled) {
                state = State.DriverControlled;
            }
            if (assistantController.left_bumper) {
                power *= 0.2;
            }
            goToHeight((int)clamp((float)(liftHeight + 1 + Math.floor(power * LIFT_MAX_DIFF)), 0, MAX_HEIGHT));
        }

        double fourBarPosition = rightFourBar.getPosition();
        double change = -assistantController.right_stick_y * MAX_FOUR_BAR_SPEED;
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
            rotateFourBar(clamped);
        }

        wrist.setPosition(clamp((float)(wristPosition + (assistantController.left_trigger - assistantController.right_trigger) * 0.02), 0, 1));

        telemetry.addData("Lift Height", liftHeight);
        telemetry.addData("Four bar position", fourBarPosition);
        telemetry.addData("Wrist position", wristPosition);

        if (hangPrimed) {
            telemetry.addLine("[⚠️][ HANG PRIMED ][⚠️]");
        }
    }

    public void internalUpdate() {
        liftHeight = liftLeft.getCurrentPosition();
        liftTarget = liftLeft.getTargetPosition();

        double power = liftRight.getPower();
        if ((liftTarget - liftHeight > 100 || state == State.Hanging) && power != 1) {
            liftRight.setPower(1);
        }
        else if (liftTarget - liftHeight < 100 && state != State.Hanging && power != 0) {
            liftRight.setPower(0);
        }

        robot.logger.log("Lift Height: " + liftHeight + ", Lift Target: " + liftTarget + ", Right Lift Power: " + liftRight.getPower() + ", Right Lift Velocity: " + liftRight.getVelocity());
    }

    public void transfer() {
        if (state != State.Transferring) {
            state = State.Transferring;
            runPreset(Presets.PRE_TRANSFER);
            robot.intake.extendTo(Intake.ExtenderPosition.IN);
            robot.intake.state = Intake.State.RunningMacro;
            delaySystem.createConditionalDelay(
                    () -> robot.intake.isExtenderIn() || state != State.Transferring,
                    () -> {
                        if (state != State.Transferring) return;

                        runPreset(Arm.Presets.TRANSFER);
                        delaySystem.createDelay(200, () -> {
                            setClawPosition(ClawPosition.CLOSED);
                            robot.cancelMacro();
                        });
                    }
            );
        }
    }
    
    public void transfer(Runnable callback) {
        if (state != State.Transferring) {
            state = State.Transferring;
            runPreset(Presets.PRE_TRANSFER);
            robot.intake.extendTo(Intake.ExtenderPosition.IN);
            delaySystem.createConditionalDelay(
                    robot.intake::isExtenderIn,
                    () -> {
                        runPreset(Presets.TRANSFER);
                        delaySystem.createConditionalDelay(
                            () -> liftHeight <= Height.TRANSFER + 20,
                            () -> {
                                setClawPosition(ClawPosition.CLOSED);
                                state = State.DriverControlled;
                                delaySystem.createDelay(250, callback);
                        });
                    }
            );
        }
    }

    public void dropSample(Runnable callback) {
        rotateFourBar(FourBarPosition.STRAIGHT_FORWARD);
        rotateWrist(WristPosition.STRAIGHT);
        delaySystem.createDelay(500, () -> openClaw(callback));
    }

    public void runPreset(Preset preset) {
        goToHeight(preset.liftPosition);
        rotateFourBar(preset.fourBarPosition);
        rotateWrist(preset.wristPosition);
        setClawPosition(preset.clawPosition);
    }

    public static class Presets {
        public static final Preset RESET = new Preset(Height.DOWN, FourBarPosition.STRAIGHT_UP, 0.1, ClawPosition.CLOSED);
        public static final Preset PRE_TRANSFER = new Preset(Height.PRE_TRANSFER, FourBarPosition.STRAIGHT_BACK, WristPosition.TRANSFER, ClawPosition.OPEN);
        public static final Preset TRANSFER = new Preset(Height.TRANSFER, FourBarPosition.STRAIGHT_BACK, WristPosition.TRANSFER, ClawPosition.OPEN);
        public static final Preset PRE_SAMPLE_DEPOSIT = new Preset(Height.UPPER_BUCKET, FourBarPosition.STRAIGHT_UP, WristPosition.STRAIGHT, ClawPosition.CLOSED);
        public static final Preset SPECIMEN_GRAB = new Preset(Height.WALL_PICKUP, FourBarPosition.STRAIGHT_BACK, WristPosition.STRAIGHT, ClawPosition.OPEN);
        public static final Preset PRE_SPECIMEN_DEPOSIT = new Preset(Height.PRE_SPECIMEN_DEPOSIT, FourBarPosition.SPECIMEN_DEPOSIT, WristPosition.SPECIMEN_DEPOSIT, ClawPosition.CLOSED);
        public static final Preset SPECIMEN_DEPOSIT = new Preset(Height.SPECIMEN_DEPOSIT, FourBarPosition.SPECIMEN_DEPOSIT, WristPosition.SPECIMEN_DEPOSIT, ClawPosition.CLOSED);
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