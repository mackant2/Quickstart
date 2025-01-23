package components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.DelaySystem;
import utils.EnhancedColorSensor;
import utils.ParsedHardwareMap;
import utils.Robot;

public class Intake {
    public enum IntakeState {
        DriverControlled,
        Intaking,
        Transferring,
        Rejecting,
        ResetTransfer
    }
    static class GatePosition {
        public static final int OPEN = 1;
        public static final double CLOSED = 0;
    }
    public static class ExtenderPosition {
        public static final int IN = 30;
        public static final int OUT = 2000;
    }
    public static class FlipdownPosition {
        public static final int in = 0;
        public static final int out = 1;
    }
    public IntakeState state = IntakeState.DriverControlled;
    DelaySystem delaySystem = new DelaySystem();
    DcMotorEx intake, extender;
    ColorSensor leftColorSensor, rightColorSensor;
    DistanceSensor leftDistanceSensor, rightDistanceSensor;
    Servo flipdown, gate;
    RevBlinkinLedDriver display;
    Robot robot;
    Gamepad driverController;
    StateMachine stateMachine;
    float transferStartTime;
    boolean transferStarted = false;
    boolean transferResetStarted = false;

    public Intake(ParsedHardwareMap drive, Robot robot) {
        this.robot = robot;
        intake = drive.intake;
        leftColorSensor = drive.leftColorSensor;
        rightColorSensor = drive.rightColorSensor;
        leftDistanceSensor = drive.leftDistanceSensor;
        rightDistanceSensor = drive.rightDistanceSensor;
        flipdown = drive.flipDown;
        gate = drive.gate;
        display = drive.display;
        extender = drive.extender;
        driverController = robot.opMode.gamepad1;

        stateMachine = new StateMachineBuilder()
            .state(IntakeState.DriverControlled)
            .transition(() -> driverController.right_bumper)
            .build();
    }

    public void Initialize() {
        //Move intake to flipped up and in
        flipdown.setPosition(0);
        extender.setTargetPosition(1000);
        delaySystem.CreateDelay(500, () -> extender.setTargetPosition(ExtenderPosition.IN));
    }

    public void ToggleFlipdown() {
        flipdown.setPosition(flipdown.getPosition() == FlipdownPosition.in ? FlipdownPosition.out : FlipdownPosition.in);
    }

    public void SetIntakeState(IntakeState newState) {
        state = state == newState ? IntakeState.DriverControlled : newState;
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        switch (state) {
            case Intaking:
               // gate.setPosition(GatePosition.CLOSED);
               // intake.setPower(-1);
                intake.setPower(-1);
                if (!driverController.dpad_right) {
                    state = Intake.IntakeState.DriverControlled;
                }
                //Again, if intake has a sample (not white)
                if (EnhancedColorSensor.CheckSensor(rightColorSensor, rightDistanceSensor, EnhancedColorSensor.Color.Any)) {
                    delaySystem.CreateDelay(500, () -> intake.setPower(0));
                }
                break;
            case Transferring:
                extender.setTargetPosition(ExtenderPosition.IN);
                if (extender.getCurrentPosition() <= ExtenderPosition.IN) {
                    if (!transferStarted) {
                        transferStarted = true;
                        transferStartTime = System.currentTimeMillis();
                        gate.setPosition(GatePosition.OPEN);
                        intake.setPower(-1);
                        delaySystem.CreateDelay(2000, () -> {
                            if (state == IntakeState.Transferring && !robot.transferPlate.sampleIsPresent) {
                                transferStarted = false;
                                state = IntakeState.ResetTransfer;
                            }
                        });
                    }

                    if (robot.transferPlate.sampleIsPresent) {
                        transferStarted = false;
                        state = IntakeState.DriverControlled;
                    }
                }
                break;
            case DriverControlled:
                gate.setPosition(GatePosition.CLOSED);
              /*  double power = driverController.right_trigger - driverController.left_trigger;
                if (power != 0) {
                    int currentPosition = extender.getCurrentPosition();
                    int target = currentPosition + (int)Math.round(power * 500);
                    extender.setTargetPosition((int)clamp(target, ExtenderPosition.IN, ExtenderPosition.OUT));
                }

                if (intake.getPower() != 0) {
                    intake.setPower(0);
                }*/
                if (driverController.dpad_right) {
                    state = Intake.IntakeState.Intaking;
                }
                else if (driverController.dpad_left) {
                    state = Intake.IntakeState.Rejecting;
                }
                intake.setPower(0);
                break;
            case Rejecting:
                gate.setPosition(GatePosition.OPEN);
                intake.setPower(1);
                if (!driverController.dpad_left) {
                    state = Intake.IntakeState.DriverControlled;
                }

               // intake.setPower(1);
                break;
            case ResetTransfer:
                if (!transferResetStarted) {
                    transferResetStarted = true;
                    intake.setPower(1);
                    delaySystem.CreateDelay(1000, () -> {
                        transferResetStarted = false;
                        state = IntakeState.Transferring;
                    });
                }
                break;
        }

        //Hard stop so intake doesn't flip down while in
        if (extender.getCurrentPosition() < 300 && state == IntakeState.Intaking) {
            state = IntakeState.DriverControlled;
        }

        delaySystem.Update();

        robot.opMode.telemetry.addData("Intake State", state);
        robot.opMode.telemetry.addData("Extender Voltage (MILLIAMPS)", extender.getCurrent(CurrentUnit.MILLIAMPS));
        robot.opMode.telemetry.addData("Extender Velocity", extender.getVelocity());
        robot.opMode.telemetry.addData("Extender Position", extender.getCurrentPosition());
        robot.opMode.telemetry.addData("Extender Target Position", extender.getTargetPosition());
        robot.opMode.telemetry.addData("Extender Power", extender.getPower());
    }
}
