package tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import components.Intake;
import utils.ParsedHardwareMap;


@TeleOp (group = "tuning", name = "[TUNING] Intake")
public class IntakeTuner extends OpMode {
    enum TunerState {
        Extending,
        Tuning,
        ResettingEncoder,
        Completed
    }

    DcMotorEx extender;
    Servo flipDown;
    TouchSensor limiter, intakeLimiter;
    TunerState state = TunerState.Extending;

    @Override
    public void init() {
        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        extender = parsedHardwareMap.extender;
        flipDown = parsedHardwareMap.flipDown;
        limiter = parsedHardwareMap.extenderLimiter;
        intakeLimiter = parsedHardwareMap.intakeLimiter;

        flipDown.setPosition(Intake.FlipdownPosition.UP);
    }

    @Override
    public void loop() {
        flipDown.setPosition(gamepad1.y ? 1 : 0);
        switch (state) {
            case Extending:
                telemetry.addLine("Extend intake with right trigger and press A/X when done");
                extender.setTargetPosition(extender.getCurrentPosition() + Math.round(gamepad1.right_trigger * 50));
                if (gamepad1.a) {
                    state = TunerState.Tuning;
                }
                break;
            case Tuning:
                extender.setTargetPosition(extender.getCurrentPosition() - 15);
                if (limiter.isPressed()) {
                    state = TunerState.ResettingEncoder;
                }
                break;
            case ResettingEncoder:
                if (extender.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (extender.getCurrentPosition() == 0) {
                    state = TunerState.Completed;
                    requestOpModeStop();
                }
                break;
        }
        telemetry.addData("State", state);
        telemetry.addData("Intake Limiter Pressed", intakeLimiter.isPressed());
    }
}