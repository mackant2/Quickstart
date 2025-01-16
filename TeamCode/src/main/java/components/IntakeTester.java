package components;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import utils.ParsedHardwareMap;
import utils.PressEventSystem;
import utils.Robot;

@TeleOp(name = "Intaketester", group = "official")
public class IntakeTester extends OpMode {
    Gamepad driverController;
    Gamepad assistantController;
    Robot robot;
    ColorSensor intakeColorSensor;

    @Override
    public void init() {}

    @Override
    public void loop() {
        int red1 = intakeColorSensor.red();
        int green1 = intakeColorSensor.green();
        int blue1 = intakeColorSensor.blue();


        float[] hsvValues1 = new float[3];
        Color.RGBToHSV(red1, green1, blue1, hsvValues1);
        float hue1 = hsvValues1[0];
        float saturation1 = hsvValues1[1];
        float value1 = hsvValues1[2];
        robot = new Robot(this, hardwareMap, true);


        float minSaturation = 0.4f; // Adjust as necessary
        float minValue = 0.4f; // Adjust as necessary

        if (saturation1 >= minSaturation && value1 >= minValue) {
            if (hue1 >= 0 && hue1 < 65) {
                telemetry.addData("Color 1", "Red");
                robot.intake.SetIntakeState((Intake.IntakeState.DriverControlled));
            } else if (hue1 >= 65 && hue1 < 100) {
                telemetry.addData("Color 1", "Yellow");
                robot.intake.SetIntakeState((Intake.IntakeState.DriverControlled));
            } else if (hue1 >= 165 && hue1 < 240) {
                telemetry.addData("Color 1", "Blue");
                robot.intake.SetIntakeState((Intake.IntakeState.DriverControlled));
            } else {
                telemetry.addData("Color 1", "Unknown");
                robot.intake.SetIntakeState((Intake.IntakeState.DriverControlled));

            }
        } else {
            telemetry.addData("Color 1", "Unknown");
        }


        if(gamepad1.dpad_left){
            robot.intake.SetIntakeState((Intake.IntakeState.Intaking));
        }
        if(gamepad1.dpad_right){
            robot.intake.SetIntakeState((Intake.IntakeState.Rejecting));
        }
    }
}