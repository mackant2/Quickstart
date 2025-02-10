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
    TouchSensor intakeLimiter;
    boolean didStateAction = false;

    public Intake(Robot robot) {
        ParsedHardwareMap parsedHardwareMap = robot.parsedHardwareMap;

        this.robot = robot;
        intake = parsedHardwareMap.intake;
        flipdown = parsedHardwareMap.flipDown;
        display = parsedHardwareMap.display;
        extender = parsedHardwareMap.extender;
        driverController = robot.opMode.gamepad1;
        intakeLimiter = parsedHardwareMap.intakeLimiter;

        stateMachine = new StateMachineBuilder()
            .state(State.DriverControlled)
            .transition(() -> driverController.right_bumper)
            .build();
    }

    public void Initialize() {
        //Move intake to flipped up and in
        Reset();
    }

    public void ToggleFlipdown() {
        flipdown.setPosition(flipdown.getPosition() == FlipdownPosition.UP ? FlipdownPosition.DOWN : FlipdownPosition.UP);
    }

    public int GetExtenderPosition() {
        return extender.getCurrentPosition();
    }

    public void AdjustExtenderPosition(int change) {
        ExtendTo(extender.getCurrentPosition() + change);
    }

    public void ExtendTo(int position) {
        SetFlipdownPosition(FlipdownPosition.UP);
        extender.setTargetPosition(position);
    }

    public void SetFlipdownPosition(double position) {
        flipdown.setPosition(position);
    }

    public void RunIntake(IntakeDirection direction) {
        if (direction == IntakeDirection.Intaking && intakeLimiter.isPressed()) return;

        intake.setPower(direction == IntakeDirection.Intaking ? 1 : -1);
    }

    public boolean isExtenderIn() {
        return intakeLimiter.isPressed();
    }

    public void StopIntake() {
        intake.setPower(0);
    }

    public void Reset() {
        flipdown.setPosition(FlipdownPosition.UP);
        extender.setTargetPosition(ExtenderPosition.IN);
    }

    public void SetIntakeState(State newState) {
        state = state == newState ? State.DriverControlled : newState;
    }

    public IntakeDirection getIntakeDirection() {
        return intake.getPower() == 1 ? IntakeDirection.Intaking : IntakeDirection.Rejecting;
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }
    //int currentPosition = extender.getCurrentPosition();

    public void Update() {
        switch (state) {
            case DriverControlled:
                if (driverController.right_trigger > .05) {
                    int target = extender.getCurrentPosition() + (int)Math.round(driverController.right_trigger * 500);
                    extender.setTargetPosition((int)clamp(target, ExtenderPosition.IN, ExtenderPosition.OUT));

                }
                else if (driverController.left_trigger > .05) {
                    int target = extender.getCurrentPosition() + (int)Math.round(-driverController.left_trigger * 500);
                    extender.setTargetPosition((int)clamp(target, ExtenderPosition.IN, ExtenderPosition.OUT));

                }
                intake.setPower(0);
                break;
            case Transferring:
                flipdown.setPosition(FlipdownPosition.UP);
                extender.setTargetPosition(ExtenderPosition.IN);
                if (extender.getCurrentPosition() > ExtenderPosition.IN + 30 && !didStateAction) {
                    didStateAction = true;
                    robot.delaySystem.CreateDelay(2000, () -> {
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
            StopIntake();
        }

        robot.opMode.telemetry.addData("Intake State", state);
        robot.opMode.telemetry.addData("Intake Power", intake.getPower());
        robot.opMode.telemetry.addData("Extender Voltage (MILLIAMPS)", extender.getCurrent(CurrentUnit.MILLIAMPS));
        robot.opMode.telemetry.addData("Extender Velocity", extender.getVelocity());
        robot.opMode.telemetry.addData("Extender Position", extender.getCurrentPosition());
        robot.opMode.telemetry.addData("Extender Target Position", extender.getTargetPosition());
        robot.opMode.telemetry.addData("Extender Power", extender.getPower());
    }
}
