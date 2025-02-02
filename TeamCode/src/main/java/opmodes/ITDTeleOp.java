package opmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import components.Arm;
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
        pressEventSystem = new PressEventSystem(telemetry);
        otos = parsedHardwareMap.myOtos;

        configureOtos();
    }

    @Override
    public void start() {
        //lift & four bar
        pressEventSystem.AddListener(assistantController, "right_bumper", robot.arm::ToggleClaw);
        pressEventSystem.AddListener(assistantController, "y", () -> robot.arm.GoToHeight(Arm.Height.UPPER_BUCKET));
        pressEventSystem.AddListener(assistantController, "x", () -> robot.arm.PresetSpecScore());
        pressEventSystem.AddListener(assistantController, "a", robot.arm::PrepareToGrabSpecimen);
        pressEventSystem.AddListener(assistantController, "dpad_right", robot.arm::PrepareToDepositSpecimen);
        pressEventSystem.AddListener(assistantController, "dpad_down", robot.arm::UpdateWallPickupHeight);
        pressEventSystem.AddListener(assistantController, "dpad_left", robot.arm::TransferPreset);
       /* pressEventSystem.AddListener(assistantController, "dpad_left", () -> {
            robot.intake.state = Intake.IntakeState.Transferring;
            robot.arm.state = Arm.ArmState.Transferring;
        });
*/
        //drivetrain & intake
        pressEventSystem.AddListener(driverController, "y", robot.intake::ToggleFlipdown);
        pressEventSystem.AddListener(driverController, "a", robot.drivetrain::resetOrientation);
      //  pressEventSystem.AddListener(driverController, "left_bumper", () -> robot.intake.SetIntakeState(Intake.IntakeState.Intaking));
   //     pressEventSystem.AddListener(driverController, "right_bumper", () -> robot.intake.SetIntakeState(Intake.IntakeState.Rejecting));
        pressEventSystem.AddListener(driverController, "dpad_up", robot.drivetrain::ToggleHalfSpeed);
      //  pressEventSystem.AddListener(driverController, "right_trigger", () -> robot.intake.SetIntakeState(Intake.IntakeState.Extending));
      //  pressEventSystem.AddListener(driverController, "left_trigger", () -> robot.intake.SetIntakeState(Intake.IntakeState.Retracting));

        robot.parsedHardwareMap.flipDown.setPosition(0);
        robot.parsedHardwareMap.extender.setTargetPosition(Intake.ExtenderPosition.IN);
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