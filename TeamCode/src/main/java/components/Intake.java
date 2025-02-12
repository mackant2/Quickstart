package components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.ParsedHardwareMap;
import utils.Robot;

public class Intake {
    public enum State {
        DriverControlled,
        Intaking,
        Rejecting,
        Transferring
    }
    public enum IntakeDirection {
        Intaking,
        Rejecting
    }
    public static class ExtenderPosition {
        public static final int IN = 0;
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
    StateMachine stateMachine;
    TouchSensor intakeLimiter, extenderLimiter;
    boolean didStateAction = false;
    public int extenderPosition;
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
        intakeLimiter = parsedHardwareMap.intakeLimiter;

        stateMachine = new StateMachineBuilder()
            .state(State.DriverControlled)
            .transition(() -> driverController.right_bumper)
            .build();
    }

    public void toggleFlipdown() {
        flipdown.setPosition(flipdown.getPosition() == FlipdownPosition.UP ? FlipdownPosition.DOWN : FlipdownPosition.UP);
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

    public void runMarchingIntake(int startPosition, Runnable callback, Runnable failedCallback) {
        extendTo(startPosition);
        robot.delaySystem.createConditionalDelay(
                () -> extenderPosition >= startPosition - 10,
                () -> {
                    robot.intake.setExtenderVelocity(200);
                    robot.intake.extendTo(ExtenderPosition.OUT);
                    SetFlipdownPosition(FlipdownPosition.DOWN);
                    RunIntake(IntakeDirection.Intaking);
                    robot.delaySystem.createConditionalDelay(
                            () -> this.hasSample() || extenderPosition >= ExtenderPosition.OUT - 10,
                        () -> {
                            if (this.hasSample()) {
                                stopIntake();
                                robot.intake.setExtenderVelocity(1000);
                                callback.run();
                            }
                            else {
                                stopIntake();
                                robot.intake.setExtenderVelocity(1000);
                                failedCallback.run();
                            }
                        }
                    );
                }
        );
    }

    public void SetFlipdownPosition(double position) {
        flipdown.setPosition(position);
    }

    public void RunIntake(IntakeDirection direction) {
        if (direction == IntakeDirection.Intaking && intakeLimiter.isPressed()) return;

        intake.setPower(direction == IntakeDirection.Intaking ? 1 : -1);
    }

    public boolean isExtenderIn() {
        return extenderPosition <= ExtenderPosition.IN + 20;
    }

    public boolean hasSample() {
        return intakeLimiter.isPressed();
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void reset() {
        flipdown.setPosition(FlipdownPosition.UP);
        extendTo(ExtenderPosition.IN);
    }

    public void setExtenderVelocity(int velocity) {
        extender.setVelocity(velocity);
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void update() {
        switch (state) {
            case DriverControlled:
                if (driverController.right_trigger > .05) {
                    int target = extenderPosition + (int)Math.round(driverController.right_trigger * 500);
                    extendTo(((int)clamp(target, ExtenderPosition.IN, ExtenderPosition.OUT)));

                }
                else if (driverController.left_trigger > .05) {
                    int target = extenderPosition + (int)Math.round(-driverController.left_trigger * 500);
                    extendTo((int)clamp(target, ExtenderPosition.IN, ExtenderPosition.OUT));

                }
                intake.setPower(0);
                break;
            case Transferring:
                flipdown.setPosition(FlipdownPosition.UP);
                extendTo(ExtenderPosition.IN);
                if (extenderPosition > ExtenderPosition.IN + 30 && !didStateAction) {
                    didStateAction = true;
                    robot.delaySystem.createDelay(2000, () -> {
                        didStateAction = false;
                        state = State.DriverControlled;
                    });
                }
        }

        if (driverController.left_bumper) {
            RunIntake(IntakeDirection.Rejecting);
        }
        else if (driverController.right_bumper && !intakeLimiter.isPressed()) {
            RunIntake(IntakeDirection.Intaking);
        }
        else if (intake.getPower() != 0) {
            stopIntake();
        }

        robot.opMode.telemetry.addData("Intake State", state);
        robot.opMode.telemetry.addData("Intake Power", intake.getPower());
        robot.opMode.telemetry.addData("Extender Voltage (MILLIAMPS)", extender.getCurrent(CurrentUnit.MILLIAMPS));
        robot.opMode.telemetry.addData("Extender Velocity", extender.getVelocity());
        robot.opMode.telemetry.addData("Extender Position", extenderPosition);
        robot.opMode.telemetry.addData("Extender Target Position", extenderTarget);
        robot.opMode.telemetry.addData("Extender Power", extender.getPower());
    }

    public void internalUpdate() {
        extenderPosition = extender.getCurrentPosition();
        extenderTarget = extender.getTargetPosition();
    }
}
