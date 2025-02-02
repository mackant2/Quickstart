package components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.ParsedHardwareMap;
import utils.Robot;

public class Intake {
    public enum IntakeState {
        DriverControlled,
        Intaking,
        Rejecting,
        Transferring
    }
    public static class ExtenderPosition {
        public static final int IN = 0;
        public static final int OUT = 2000;
    }
    public static class FlipdownPosition {
        public static final int UP = 0;
        public static final int DOWN = 1;
    }
    public IntakeState state = IntakeState.DriverControlled;
    DcMotorEx intake, extender;
    Servo flipdown;
    RevBlinkinLedDriver display;
    Robot robot;
    Gamepad driverController;
    StateMachine stateMachine;
    boolean didStateAction = false;

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
        Reset();
    }



    public void ToggleFlipdown() {
        flipdown.setPosition(flipdown.getPosition() == FlipdownPosition.UP ? FlipdownPosition.DOWN : FlipdownPosition.UP);
    }

    public void Reset() {
        flipdown.setPosition(FlipdownPosition.UP);
        extender.setTargetPosition(ExtenderPosition.IN);
    }

    public void SetIntakeState(IntakeState newState) {
        state = state == newState ? IntakeState.DriverControlled : newState;
    }
    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }
    //int currentPosition = extender.getCurrentPosition();

    public void Update() {
        switch (state) {
            case Intaking:
                intake.setPower(-.1);
                if (!driverController.left_bumper) {
                    state = Intake.IntakeState.DriverControlled;
                }
                break;
            case DriverControlled:
                if (driverController.right_bumper) {
                    state = Intake.IntakeState.Rejecting;
                }
                else if (driverController.left_bumper) {
                    state = Intake.IntakeState.Intaking;

                }
                else if (driverController.right_trigger > .05) {
                    int targetout = extender.getCurrentPosition() + (int)Math.round(driverController.right_trigger * 500);
                    extender.setTargetPosition((int)clamp(targetout, ExtenderPosition.IN, ExtenderPosition.OUT));

                }
                else if (driverController.left_trigger > .05) {
                    int targetin = extender.getCurrentPosition() + (int)Math.round(-driverController.left_trigger * 500);
                    extender.setTargetPosition((int)clamp(targetin, ExtenderPosition.IN, ExtenderPosition.OUT));

                }
                intake.setPower(0);
                break;
            case Rejecting:
                intake.setPower(.1);
                if (!driverController.right_bumper) {
                    state = Intake.IntakeState.DriverControlled;
                }
                break;
            case Transferring:
                flipdown.setPosition(FlipdownPosition.UP);
                extender.setTargetPosition(ExtenderPosition.IN);
                if (extender.getCurrentPosition() > ExtenderPosition.IN + 30 && !didStateAction) {
                    didStateAction = true;
                    robot.delaySystem.CreateDelay(2000, () -> {
                        didStateAction = false;
                        state = IntakeState.DriverControlled;
                    });
                }
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
