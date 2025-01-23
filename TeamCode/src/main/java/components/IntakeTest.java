package components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import utils.DelaySystem;
import utils.RobotTest;
import utils.TestHardwareMap;

public class IntakeTest {
    public enum IntakeTestState {
        DriverControlled,
        Intaking,
        Transferring,
        Rejecting,
        ResetTransfer

    }
    public IntakeTestState teststate = IntakeTestState.DriverControlled;

    DelaySystem delaySystem = new DelaySystem();
    DcMotorEx intaketest;
    ColorSensor intakeColorSensor;
    RobotTest robotTest;
    Gamepad driverController;
    StateMachine stateMachine1;
    float transferStartTime;
    boolean transferStarted = false;
    boolean transferResetStarted = false;
    public void Initialize() {
    }
    public IntakeTest(TestHardwareMap drive, RobotTest robot) {
        this.robotTest = robot;

        driverController = robot.opMode.gamepad1;

        intaketest = drive.intaketest;

       stateMachine1 = new StateMachineBuilder()
                .state(IntakeTestState.DriverControlled)
                .transition(() -> driverController.right_bumper)
                .build();
    }


    public void SetIntakeState(IntakeTestState newTestState) {
        teststate = teststate == newTestState ? IntakeTestState.DriverControlled : newTestState;
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        switch (teststate) {
            case Intaking:
                intaketest.setPower(-1);
                if (!driverController.dpad_right) {
                    teststate = IntakeTestState.DriverControlled;
                }
                break;
            case DriverControlled:
                if (driverController.dpad_right) {
                    teststate = IntakeTestState.Intaking;
                }
                else if (driverController.dpad_left) {
                    teststate = IntakeTestState.Rejecting;
                }
                intaketest.setPower(0);
                break;
            case Rejecting:
                intaketest.setPower(1);
                if (!driverController.dpad_left) {
                    teststate = IntakeTestState.DriverControlled;
                }
                break;
        }



        delaySystem.Update();

        robotTest.opMode.telemetry.addData("Intake State", teststate);
    }
}
