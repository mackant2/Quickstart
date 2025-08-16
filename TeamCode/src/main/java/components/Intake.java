package components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import utils.ParsedHardwareMap;
import utils.Robot;

public class Intake {
    public enum State {
        DriverControlled,
        RunningMacro
    }
    public enum IntakeDirection {
        Intaking,
        Rejecting
    }
    public static class ExtenderPosition {
        public static final int IN = -10;
        public static final int OUT = 2000;
    }
    public static class FlipdownPosition {
        public static final int UP = 0;
        public static final int DOWN = 1;
    }
    public State state = State.DriverControlled;
    DcMotorEx intake, extender;
    Servo flipdown;
    RevBlinkinLedDriver display;
    Robot robot;
    Gamepad driverController;
    TouchSensor sampleSensor, extenderLimiter;
    public int extenderPosition;
    public boolean hasSample = false;
    int extenderTarget;

    public Intake(Robot robot) {
        ParsedHardwareMap parsedHardwareMap = robot.parsedHardwareMap;

        this.robot = robot;
        intake = parsedHardwareMap.intake;
        flipdown = parsedHardwareMap.flipDown;
        display = parsedHardwareMap.display;
        extender = parsedHardwareMap.extender;
        driverController = robot.opMode.gamepad1;
        extenderLimiter = parsedHardwareMap.extenderLimiter;
        sampleSensor = parsedHardwareMap.intakeLimiter;
    }

    public void toggleFlipdown() {
        if (!isInState(State.DriverControlled)) return;

        SetFlipdownPosition(flipdown.getPosition() == FlipdownPosition.UP ? FlipdownPosition.DOWN : FlipdownPosition.UP);
    }

    public void adjustExtenderPosition(int change) {
        extendTo(extenderPosition + change);
    }

    public void extendTo(int position) {
        if (extenderPosition == position) return;
        extenderPosition = position;
        SetFlipdownPosition(FlipdownPosition.UP);
        extender.setTargetPosition(extenderPosition);
    }

    boolean isInState(State checkState) {
        return state == checkState;
    }

    public void runMarchingIntake(int startPosition, Runnable callback, Runnable failedCallback) {
        state = State.RunningMacro;
        extendTo(startPosition);
        robot.delaySystem.createConditionalDelay(
                () -> extenderPosition >= startPosition - 10 || !isInState(State.RunningMacro),
                () -> {
                    if (!isInState(State.RunningMacro)) return;

                    setExtenderVelocity(400);
                    extendTo(ExtenderPosition.OUT);
                    SetFlipdownPosition(FlipdownPosition.DOWN);
                    RunIntake(IntakeDirection.Intaking);
                    robot.delaySystem.createConditionalDelay(
                        () -> hasSample || extenderPosition >= ExtenderPosition.OUT - 10 || !isInState(State.RunningMacro),
                        () -> {
                            extendTo(extenderPosition);
                            stopIntake();
                            setExtenderVelocity(1000);

                            if (!isInState(State.RunningMacro)) return;

                            if (hasSample) {
                                callback.run();
                            }
                            else {
                                failedCallback.run();
                            }

                            state = State.DriverControlled;
                        }
                    );
                }
        );
    }

    public void runMarchingIntake() {
        runMarchingIntake(extenderPosition, () -> {
            state = State.RunningMacro;
            robot.delaySystem.createConditionalDelay(
                    () -> driverController.a || !isInState(State.RunningMacro),
                    () -> {
                        if (!isInState(State.RunningMacro)) return;

                        robot.arm.transfer();
                    }
            );
        }, () -> {});
    }

    public void SetFlipdownPosition(double position) {
        flipdown.setPosition(position);
    }

    public void RunIntake(IntakeDirection direction) {
        if (direction == IntakeDirection.Intaking && sampleSensor.isPressed()) return;

        intake.setPower(direction == IntakeDirection.Intaking ? 1 : -1);
    }

    public boolean isExtenderIn() {
        return extenderPosition <= ExtenderPosition.IN + 20;
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void reset() {
        SetFlipdownPosition(FlipdownPosition.UP);
        extendTo(ExtenderPosition.IN);
    }

    public void setExtenderVelocity(int velocity) {
        extender.setVelocity(velocity);
    }
    
    int clampExtender(int target) {
        return Math.max(ExtenderPosition.IN, Math.min(target, ExtenderPosition.OUT));
    }

    public void update() {
        switch (state) {
            case DriverControlled:
                float input = driverController.right_trigger - driverController.left_trigger;
                int target = extenderPosition + Math.round(input * 500);
                extender.setTargetPosition(clampExtender(target));

                if (driverController.left_bumper) {
                    RunIntake(IntakeDirection.Rejecting);
                }
                else if (driverController.right_bumper && !sampleSensor.isPressed()) {
                    RunIntake(IntakeDirection.Intaking);
                }
                else if (intake.getPower() != 0) {
                    stopIntake();
                }
            break;
            case RunningMacro:
                robot.opMode.telemetry.addLine("[️️⚠️][ INTAKE AUTOMATED ][⚠️]");
            break;
        }

        if (hasSample) {
            robot.opMode.telemetry.addLine("[✅][ SAMPLE CAPTURED ][✅]");
        }
    }

    public void internalUpdate() {
        extenderPosition = extender.getCurrentPosition();
        extenderTarget = extender.getTargetPosition();
        hasSample = sampleSensor.isPressed();
    }
}
