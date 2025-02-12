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


@TeleOp (group = "tuning", name="[TUNING] Robot Setup")
public class RobotSetup extends OpMode {
    enum State {
        RaisingLift,
        ExtendingExtender,
        Calibrating,
        Resetting
    }
    State state = State.RaisingLift;
    String keyPosition = "transfer";
    Boolean triggerStarted = false;

    PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
    DelaySystem delaySystem = new DelaySystem();

    Robot robot;

    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    TouchSensor liftLimiter, extenderLimiter;

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
        extenderLimiter = parsedHardwareMap.extenderLimiter;
        leftFourBar = parsedHardwareMap.leftFourBar;
        rightFourBar = parsedHardwareMap.rightFourBar;
        wrist = parsedHardwareMap.wrist;
        claw = parsedHardwareMap.claw;

        robot.arm.runPreset(Arm.Presets.RESET);
    }

    @Override
    public void start() {
        pressEventSystem.addListener(gamepad1, "a", () -> claw.setPosition(claw.getPosition() == Arm.ClawPosition.Open ? Arm.ClawPosition.Closed : Arm.ClawPosition.Open));
    }

    @Override
    public void loop() {
        pressEventSystem.update();
        delaySystem.update();

        switch (state) {
            case RaisingLift: {
                telemetry.addLine("Use the right trigger to raise the lift.");

                double power = gamepad1.right_trigger;

                if (power != 0 && !triggerStarted) triggerStarted = true;

                if (triggerStarted) {
                    if (power == 0) {
                        triggerStarted = false;
                        state = State.ExtendingExtender;
                    } else {
                        robot.arm.adjustLiftHeight((int) Math.round(power * 50));
                    }
                }
                break;
            }
            case ExtendingExtender:
                    telemetry.addLine("Use the right trigger to extend the extender.");

                    double power = gamepad1.right_trigger;

                    if (power != 0 && !triggerStarted) triggerStarted = true;

                    if (triggerStarted) {
                        if (power == 0) {
                            triggerStarted = false;
                            state = State.Calibrating;
                        }
                        else {
                            robot.intake.adjustExtenderPosition(Math.round(gamepad1.right_trigger * 50));
                        }
                    }
                break;
            case Calibrating:
                telemetry.addLine("Calibrating...");
                if (!liftLimiter.isPressed()) {
                    robot.arm.adjustLiftHeight(-10);
                }
                if (!extenderLimiter.isPressed()) {
                    robot.intake.adjustExtenderPosition(-25);
                }
                if (liftLimiter.isPressed() && extenderLimiter.isPressed()) {
                    state = State.Resetting;
                }
                break;
            case Resetting:
                telemetry.addLine("Resetting encoder...");
                if (liftLeft.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.parsedHardwareMap.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (liftLeft.getCurrentPosition() == 0) {
                    requestOpModeStop();
                }
                break;
        }
    }

    @Override
    public void stop() {
        robot.logger.close();
    }
}