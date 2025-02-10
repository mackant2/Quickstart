package tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

import components.Arm;
import utils.DelaySystem;
import utils.ParsedHardwareMap;
import utils.PressEventSystem;
import utils.Robot;


@TeleOp (group = "tuning", name="[TUNING] Arm")
public class ArmTuner extends OpMode {
    enum State {
        RunPresets,
        Playground,
        Tuner,
        Calibrating,
        Resetting
    }

    State state = State.Tuner;

    PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
    DelaySystem delaySystem = new DelaySystem();

    Robot robot;

    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    TouchSensor liftLimiter;

    List<Arm.Preset> armPresets = Arrays.asList(
        Arm.Presets.RESET,
        Arm.Presets.SPECIMEN_GRAB,
        Arm.Presets.PRE_SPECIMEN_DEPOSIT,
        Arm.Presets.SPECIMEN_DEPOSIT,
        Arm.Presets.PRE_TRANSFER,
        Arm.Presets.TRANSFER,
        Arm.Presets.PRE_SAMPLE_DEPOSIT
    );
    int presetIndex = 0;
    Boolean scheduledNextPreset = false;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, false);

        ParsedHardwareMap parsedHardwareMap = robot.parsedHardwareMap;
        liftLeft = parsedHardwareMap.liftLeft;
        liftRight = parsedHardwareMap.liftRight;
        liftLimiter = parsedHardwareMap.liftLimiter;
        leftFourBar = parsedHardwareMap.leftFourBar;
        rightFourBar = parsedHardwareMap.rightFourBar;
        wrist = parsedHardwareMap.wrist;
        claw = parsedHardwareMap.claw;

        robot.arm.RunPreset(Arm.Presets.RESET);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Mode", state);
        telemetry.addLine("Press X/◻ for playground mode");
        telemetry.addLine("Press B/◯ for tuning mode");
        telemetry.addLine("Press A/X for preset testing mode");

        if (gamepad1.x) {
            state = State.Playground;
        }
        else if (gamepad1.b) {
            state = State.Tuner;
        }
        else if (gamepad1.a) {
            state = State.RunPresets;
        }
    }

    @Override
    public void start() {
        if (state == State.Playground) {
            robot.arm.RegisterControls();
        }
    }

    @Override
    public void loop() {
        pressEventSystem.Update();
        delaySystem.Update();

        switch (state) {
            case RunPresets:
                    if (!scheduledNextPreset) {
                        scheduledNextPreset = true;

                        robot.arm.RunPreset(armPresets.get(presetIndex));

                        delaySystem.CreateDelay(3000, () -> {
                            presetIndex++;
                            if (presetIndex == armPresets.size()) presetIndex = 0;
                            scheduledNextPreset = false;
                        });
                    }
                break;
            case Tuner:
                telemetry.addLine("1) Raise the arm using the right trigger");
                telemetry.addLine("2) Press A/X when you are ready to begin the automated tuning process.");
                double trigger = gamepad1.right_trigger;
                if (trigger > 0) {
                    robot.arm.AdjustLiftHeight((int)Math.round(trigger * 50));
                }
                telemetry.addData("Left Lift Position", liftLeft.getCurrentPosition());
                telemetry.addData("Left Lift Target Position", liftLeft.getTargetPosition());

                if (gamepad1.a) {
                    state = State.Calibrating;
                }
                break;
            case Calibrating:
                robot.arm.AdjustLiftHeight(-10);
                if (liftLimiter.isPressed()) {
                    state = State.Resetting;
                }
                break;
            case Resetting:
                if (liftLeft.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (liftLeft.getCurrentPosition() == 0) {
                    requestOpModeStop();
                }
                telemetry.addLine("Resetting encoder...");
                break;
        }
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}