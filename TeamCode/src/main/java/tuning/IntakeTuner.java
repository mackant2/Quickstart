package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import components.Intake;
import utils.ParsedHardwareMap;


@TeleOp (group = "tuning", name = "[TUNING] Intake")
public class IntakeTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        DcMotorEx extender = parsedHardwareMap.extender;
        Servo flipDown = parsedHardwareMap.flipDown;
        TouchSensor limiter = parsedHardwareMap.extenderLimiter;

        String state = "extending";

        flipDown.setPosition(Intake.FlipdownPosition.UP);

        while (!isStarted()) {
            telemetry.addData("limit touched", limiter.isPressed());
            telemetry.update();
        }
        while (!isStopRequested()) {
            flipDown.setPosition(gamepad1.y ? 1 : 0);
            extender.setTargetPosition(extender.getCurrentPosition() - Math.round(gamepad1.right_trigger * 10));
            if (state.equals("extending")) {
                telemetry.addLine("Extend intake with right trigger and press A/X when done");
                extender.setTargetPosition(extender.getCurrentPosition() + Math.round(gamepad1.right_trigger * 10));
                if (gamepad1.a) {
                    state = "tuning";
                }
            }
            else if (state.equals("tuning")) {
                extender.setTargetPosition(extender.getCurrentPosition() - 100);
                if (limiter.isPressed()) {
                    state = "resetting";
                }
            }
            else if (state.equals("resetting")) {
                if (extender.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (extender.getCurrentPosition() == 0) {
                    state = "completed";
                    requestOpModeStop();
                }
                telemetry.addLine("Resetting encoder...");
            }
            telemetry.addData("FlipDown Position", flipDown.getPosition());
            telemetry.update();
        }
    }
}