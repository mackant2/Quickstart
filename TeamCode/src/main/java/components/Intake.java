package components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.DelaySystem;
import utils.ParsedHardwareMap;
import utils.Robot;

public class Intake {
    public enum IntakeState {
        DriverControlled,
        Intaking,
        Rejecting
    }
    public static class ExtenderPosition {
        public static final int IN = 30;
        public static final int OUT = 2000;
    }
    public static class FlipdownPosition {
        public static final int UP = 0;
        public static final int DOWN = 1;
    }
    public IntakeState state = IntakeState.DriverControlled;
    DelaySystem delaySystem = new DelaySystem();
    DcMotorEx intake, extender;
    Servo flipdown;
    RevBlinkinLedDriver display;
    Robot robot;
    Gamepad driverController;
    StateMachine stateMachine;
    float transferStartTime;
    boolean transferStarted = false;
    boolean transferResetStarted = false;

    public Intake(Robot robot) {
        ParsedHardwareMap parsedHardwareMap = robot.parsedHardwareMap;

        this.robot = robot;
        intake = parsedHardwareMap.intake;
        flipdown = parsedHardwareMap.flipDown;
        display = parsedHardwareMap.display;
        extender = parsedHardwareMap.extender;
        driverController = robot.opMode.gamepad1;

        stateMachine = new StateMachineBuilder()
            .state(IntakeState.DriverControlled)
            .transition(() -> driverController.right_bumper)
            .build();
    }

    public void Initialize() {
        //Move intake to flipped up and in
        flipdown.setPosition(FlipdownPosition.UP);
        extender.setTargetPosition(1000);
        delaySystem.CreateDelay(500, () -> extender.setTargetPosition(ExtenderPosition.IN));
    }

    public void ToggleFlipdown() {
        flipdown.setPosition(flipdown.getPosition() == FlipdownPosition.UP ? FlipdownPosition.DOWN : FlipdownPosition.UP);
    }

    public void SetIntakeState(IntakeState newState) {
        state = state == newState ? IntakeState.DriverControlled : newState;
    }

    public void Update() {
        switch (state) {
            case Intaking:
                intake.setPower(-1);
                if (!driverController.dpad_left) {
                    state = Intake.IntakeState.DriverControlled;
                }
                break;
            case DriverControlled:
                if (driverController.dpad_right) {
                    state = Intake.IntakeState.Rejecting;
                }
                else if (driverController.dpad_left) {
                    state = Intake.IntakeState.Intaking;
                }
                intake.setPower(0);
                break;
            case Rejecting:
                intake.setPower(1);
                if (!driverController.dpad_right) {
                    state = Intake.IntakeState.DriverControlled;
                }
                break;
        }

        delaySystem.Update();

        robot.opMode.telemetry.addData("Intake State", state);
        robot.opMode.telemetry.addData("Intake Power", intake.getPower());
        robot.opMode.telemetry.addData("Extender Voltage (MILLIAMPS)", extender.getCurrent(CurrentUnit.MILLIAMPS));
        robot.opMode.telemetry.addData("Extender Velocity", extender.getVelocity());
        robot.opMode.telemetry.addData("Extender Position", extender.getCurrentPosition());
        robot.opMode.telemetry.addData("Extender Target Position", extender.getTargetPosition());
        robot.opMode.telemetry.addData("Extender Power", extender.getPower());
    }
}
