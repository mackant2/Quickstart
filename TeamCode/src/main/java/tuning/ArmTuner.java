package tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import components.Arm;
import utils.DelaySystem;
import utils.ParsedHardwareMap;
import utils.PressEventSystem;
import utils.Robot;


@TeleOp (group = "tuning", name="[TUNING] Arm")
public class ArmTuner extends OpMode {
    String state = "run key positions";
    String keyPosition = "transfer";
    Boolean scheduledNextPosition = false;

    PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
    DelaySystem delaySystem = new DelaySystem();

    Robot robot;

    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    TouchSensor liftLimiter;

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

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

        robot.arm.Reset();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Mode", state);
        telemetry.addLine("Press X/◻ for playground mode");
        telemetry.addLine("Press B/◯ for encoder calibration mode");
        telemetry.addLine("Press A/X for key positions testing mode");

        if (gamepad1.x) {
            state = "playground";
        }
        else if (gamepad1.b) {
            state = "calibrator";
        }
        else if (gamepad1.a) {
            state = "run key positions";
        }
    }

    @Override
    public void start() {
        pressEventSystem.AddListener(gamepad1, "a", () -> claw.setPosition(claw.getPosition() == Arm.ClawPosition.Open ? Arm.ClawPosition.Closed : Arm.ClawPosition.Open));
    }

    @Override
    public void loop() {
        pressEventSystem.Update();
        delaySystem.Update();

        switch (state) {
            case "run key positions":
                    if (!scheduledNextPosition) {
                        scheduledNextPosition = true;

                        switch (keyPosition) {
                            case "transfer":
                                robot.arm.PrepareToTransfer();
                                break;
                            case "up":
                                robot.arm.Reset();
                                break;
                            case "specimen":
                                robot.arm.PrepareToDepositSpecimen();
                                break;
                            case "specimen pickup":
                                robot.arm.PrepareToGrabSpecimen();
                                break;
                        }

                        delaySystem.CreateDelay(3000, () -> {
                           switch (keyPosition) {
                               case "transfer":
                                   keyPosition = "up";
                                   break;
                               case "up":
                                   keyPosition = "specimen";
                                   break;
                               case "specimen":
                                   keyPosition = "specimen pickup";
                                   break;
                               case "specimen pickup":
                                   keyPosition = "transfer";
                                   break;
                           }

                            scheduledNextPosition = false;
                        });
                    }
                break;
            case "playground":
                double change = -gamepad1.right_stick_y * 0.01;
                //Math.clamp causes crash here, so using custom method
                double leftClamped = clamp((float)(leftFourBar.getPosition() + change), 0, 1);
                leftFourBar.setPosition(leftClamped);
                rightFourBar.setPosition(leftClamped);

                double power = -gamepad1.left_stick_y;
                int maxDiff = 400;
                if (!liftLimiter.isPressed() || power > 0) {
                    robot.arm.AdjustLiftHeight(1 + (int)Math.round(power * maxDiff));
                }

                robot.logger.Log(
                        "Position: " + liftLeft.getCurrentPosition() +
                                ", Position Differential (Left - Right): " + (liftLeft.getCurrentPosition() - liftRight.getCurrentPosition()) +
                                ", Power: " + liftLeft.getPower() +
                                ", Velocity: " + liftLeft.getVelocity() +
                                ", Voltage (MILLIAMPS): " + liftLeft.getCurrent(CurrentUnit.MILLIAMPS));

                double wristPower = gamepad1.right_trigger - gamepad1.left_trigger;
                if (wristPower != 0) {
                    wrist.setPosition(wrist.getPosition() + wristPower * 0.01);
                }
                telemetry.addLine("***WARNING*** The lift is not limited. Be careful with how far you move the lift.");
                telemetry.addData("Claw Position", claw.getPosition());
                telemetry.addData("Four Bar Position", leftFourBar.getPosition());
                telemetry.addData("Wrist Degrees", (wrist.getPosition() - 0.5) * 180);
                telemetry.addData("Left Lift Position", liftLeft.getCurrentPosition());
                telemetry.addData("Left Lift Target Position", liftLeft.getTargetPosition());
                telemetry.addData("Lift Differential", liftLeft.getCurrentPosition() - liftRight.getCurrentPosition());
                telemetry.addData("Left Lift Velocity", liftLeft.getVelocity());
                telemetry.addData("Left Lift Power", liftLeft.getPower());
                break;
            case "calibrator":
                telemetry.addLine("1) Raise the arm using the right trigger");
                telemetry.addLine("2) Press A/X when you are ready to begin the automated tuning process.");
                double trigger = gamepad1.right_trigger;
                if (trigger > 0) {
                    robot.arm.AdjustLiftHeight((int)Math.round(trigger * 50));
                }
                telemetry.addData("Left Lift Position", liftLeft.getCurrentPosition());
                telemetry.addData("Left Lift Target Position", liftLeft.getTargetPosition());

                if (gamepad1.a) {
                    state = "tuning";
                }
                break;
            case "tuning":
                robot.arm.AdjustLiftHeight(-10);
                if (liftLimiter.isPressed()) {
                    state = "resetting";
                }
                break;
            case "resetting":
                if (liftLeft.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (liftLeft.getCurrentPosition() == 0) {
                    state = "complete";
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