package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import utils.ParsedHardwareMap;
import utils.PressEventSystem;


@TeleOp (group = "tuning", name = "[TUNING] FourBar")
public class FourBar extends LinearOpMode {
    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        Servo leftFourBar, rightFourBar, wrist, claw;
        leftFourBar = parsedHardwareMap.leftFourBar;
        rightFourBar = parsedHardwareMap.rightFourBar;
        claw = parsedHardwareMap.claw;
        wrist = parsedHardwareMap.wrist;

        parsedHardwareMap.flipDown.setPosition(0);

        waitForStart();

        claw.setPosition(0.8);
        parsedHardwareMap.liftLeft.setTargetPosition(0);

        pressEventSystem.addListener(gamepad1, "a", () -> claw.setPosition(claw.getPosition() == 0.8 ? 0.3 : 0.8));
        pressEventSystem.addListener(gamepad1, "dpad_up", () -> {
            double newPos = rightFourBar.getPosition() + 0.01;
            leftFourBar.setPosition(newPos);
            rightFourBar.setPosition(newPos);
        });
        pressEventSystem.addListener(gamepad1, "dpad_down", () -> {
           double newPos = rightFourBar.getPosition() - 0.01;
           leftFourBar.setPosition(newPos);
           rightFourBar.setPosition(newPos);
        });
        pressEventSystem.addListener(gamepad1, "dpad_left", () -> {
            double newPos = clamp((float)(wrist.getPosition() + 0.01), 0, 1);
            wrist.setPosition(newPos);
        });
        pressEventSystem.addListener(gamepad1, "dpad_right", () -> {
            double newPos = clamp((float)(wrist.getPosition() - 0.01), 0, 1);
            wrist.setPosition(newPos);
        });

        while (!isStopRequested()) {
            pressEventSystem.update();

            double newPos = clamp((float)(rightFourBar.getPosition() + (gamepad1.left_trigger - gamepad1.right_trigger) / 100), 0, 1);
            leftFourBar.setPosition(newPos);
            rightFourBar.setPosition(newPos);
            telemetry.addData("--BINDS--", "");
            telemetry.addData("Four Bar Binds", "A - toggle claw, triggers - move fast, D-pad up/down - increment position by 0.01");
            telemetry.addData("Wrist binds", "D-pad left/right - increment position by 0.01");
            telemetry.addData("--POSITIONS--", "");
            telemetry.addData("Four Bar Position", rightFourBar.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Lift Position", parsedHardwareMap.liftLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}