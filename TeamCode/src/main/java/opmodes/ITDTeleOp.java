package opmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import components.Arm;
import components.Intake;
import utils.EnhancedColorSensor;
import utils.ParsedHardwareMap;
import utils.PressEventSystem;
import utils.Robot;

@TeleOp (name = "[OFFICIAL] TeleOp", group = "official")
public class ITDTeleOp extends OpMode {
    Gamepad driverController;
    Gamepad assistantController;
    Robot robot;

    SparkFunOTOS otos;

    ParsedHardwareMap parsedHardwareMap;
    PressEventSystem pressEventSystem;

    @Override
    public void init() {
        robot = new Robot(this, hardwareMap, true);

        driverController = gamepad1;
        assistantController = gamepad2;
        parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        pressEventSystem = new PressEventSystem(telemetry);
        otos = parsedHardwareMap.myOtos;

        configureOtos();
    }

    @Override
    public void start() {
        //lift & four bar
        pressEventSystem.AddListener(assistantController, "a",robot.arm::ToggleClaw);
        pressEventSystem.AddListener(assistantController, "x", () -> robot.arm.GoToHeight(Arm.Height.LOWER_BUCKET));
        pressEventSystem.AddListener(assistantController, "b", () -> robot.arm.GoToHeight(Arm.Height.UPPER_BAR));
        pressEventSystem.AddListener(assistantController, "y", robot.arm::PrepareToGrabSpecimen);
        pressEventSystem.AddListener(assistantController, "dpad_right", robot.arm::PrepareToDepositSpecimen);
        pressEventSystem.AddListener(assistantController, "dpad_down", robot.arm::UpdateWallPickupHeight);

        //drivetrain & intake
        pressEventSystem.AddListener(driverController, "right_bumper", robot.intake::ToggleFlipdown);
        pressEventSystem.AddListener(driverController, "y", () -> {
            if (EnhancedColorSensor.CheckSensor(parsedHardwareMap.rightColorSensor, parsedHardwareMap.rightDistanceSensor, EnhancedColorSensor.Color.Any)) {
                robot.arm.stateMachine.setState(Arm.ArmState.InitiatingTransfer);
            }
        });
        pressEventSystem.AddListener(driverController, "dpad_up", robot.drivetrain::resetOrientation);
        pressEventSystem.AddListener(driverController, "dpad_left", () -> robot.intake.SetIntakeState(Intake.IntakeState.Intaking));
        pressEventSystem.AddListener(driverController, "dpad_right", () -> robot.intake.SetIntakeState(Intake.IntakeState.Rejecting));
    }

    @Override
    public void loop() {
        //Update utils
        pressEventSystem.Update();
        //Update components
        robot.Update();
    }

    @Override
    public void stop() {
        robot.logger.close();
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured!");
    }
}