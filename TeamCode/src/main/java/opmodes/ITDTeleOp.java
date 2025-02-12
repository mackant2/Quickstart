package opmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import components.Intake;
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
        pressEventSystem = robot.pressEventSystem;
        otos = parsedHardwareMap.myOtos;

        configureOtos();
    }

    @Override
    public void start() {
        robot.arm.registerControls();
        //drivetrain & intake - gamepad1
        pressEventSystem.addListener(driverController, "y", robot.intake::toggleFlipdown);
        pressEventSystem.addListener(driverController, "a", robot.drivetrain::resetOrientation);
        pressEventSystem.addListener(driverController, "dpad_down", robot.arm::transfer);
        pressEventSystem.addListener(driverController, "dpad_up", robot.drivetrain::ToggleHalfSpeed);

        robot.parsedHardwareMap.flipDown.setPosition(0);
        robot.intake.extendTo(Intake.ExtenderPosition.IN);
    }

    @Override
    public void loop() {
        long startTime = System.nanoTime();
        robot.update();
        telemetry.addData("Loop Time (ms)", System.nanoTime() / startTime / 1_000_000);
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