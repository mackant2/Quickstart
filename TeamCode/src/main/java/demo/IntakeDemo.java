package demo;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "[DEMO] Intake", group = "demo")
public class IntakeDemo extends LinearOpMode {

    private ColorSensor colorSensor1;
    private DistanceSensor sensorDistance1;
    private ColorSensor colorSensor2;
    private DistanceSensor sensorDistance2;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private DcMotor intakeMotor;
    private Servo flipdown;



    @Override
    public void runOpMode() {
        // Initialize the color sensor
        colorSensor1 = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "leftColorSensor"); // Initialize the second color sensor and distance
        colorSensor2 = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        flipdown = hardwareMap.get(Servo.class, "flipdown");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        flipdown.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        flipdown.setPosition(1);

        while (opModeIsActive()) {

            int red1 = colorSensor1.red();
            int green1 = colorSensor1.green();
            int blue1 = colorSensor1.blue();

            // Get the color values from the second sensor
            int red2 = colorSensor2.red();
            int green2 = colorSensor2.green();
            int blue2 = colorSensor2.blue();

            // Convert RGB to HSV for the first sensor
            float[] hsvValues1 = new float[3];
            Color.RGBToHSV(red1, green1, blue1, hsvValues1);
            float hue1 = hsvValues1[0];
            float saturation1 = hsvValues1[1];
            float value1 = hsvValues1[2];

            // Convert RGB to HSV for the second sensor
            float[] hsvValues2 = new float[3];
            Color.RGBToHSV(red2, green2, blue2, hsvValues2);
            float hue2 = hsvValues2[0];
            float saturation2 = hsvValues2[1];
            float value2 = hsvValues2[2];

            float minSaturation = 0.4f; // Adjust as necessary
            float minValue = 0.4f; // Adjust as necessary

            if (saturation1 >= minSaturation && value1 >= minValue) {
                if (hue1 >= 0 && hue1 < 65) {
                    telemetry.addData("Color 1", "Red");
                } else if (hue1 >= 65 && hue1 < 100) {
                    telemetry.addData("Color 1", "Yellow");
                } else if (hue1 >= 165 && hue1 < 240) {
                    telemetry.addData("Color 1", "Blue");
                } else {
                    telemetry.addData("Color 1", "Unknown");
                }
            } else {
                telemetry.addData("Color 1", "Unknown");
            }

            // Check for colors based on hue for the second sensor
            if (saturation2 >= minSaturation && value2 >= minValue) {
                if (hue2 >= 0 && hue2 < 65) {
                    telemetry.addData("Color 2", "Red");
                } else if (hue2 >= 65 && hue2 < 100) {
                    telemetry.addData("Color 2", "Yellow");
                } else if (hue2 >= 165 && hue2 < 240) {
                    telemetry.addData("Color 2", "Blue");
                } else {
                    telemetry.addData("Color 2", "Unknown");
                }
            } else {
                telemetry.addData("Color 2", "Unknown");
            }

            boolean color1Detected = saturation1 >= minSaturation && value1 >= minValue;
            boolean color2Detected = saturation2 >= minSaturation && value2 >= minValue;

            // Check if both sensors see the same color
            if (color1Detected && color2Detected && Math.abs(hue1 - hue2) < 15) {
                if (hue1 >= 0 && hue1 < 65) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                } else if (hue1 >= 65 && hue1 < 100) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                } else if (hue1 >= 165 && hue1 < 240) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                } else {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            } else {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }


            telemetry.addData("Current hue 1", hue1);
            telemetry.addData("Current saturation 1", saturation1);
            telemetry.addData("Current value 1", value1);
            telemetry.addData("Distance 1 (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance1.getDistance(DistanceUnit.CM)));

            telemetry.addData("Current hue 2", hue2);
            telemetry.addData("Current saturation 2", saturation2);
            telemetry.addData("Current value 2", value2);
            telemetry.addData("Distance 2 (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance2.getDistance(DistanceUnit.CM)));

            telemetry.update();

            AtomicBoolean stopping = new AtomicBoolean(false);
            if (sensorDistance1.getDistance(DistanceUnit.CM) > 3 && sensorDistance2.getDistance(DistanceUnit.CM) > 3) {
                intakeMotor.setPower(-1);
            } else if (!stopping.get()) {
                stopping.set(true);
                new Thread(() -> {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    if (Math.abs(hue1 - hue2) < 20) {
                        intakeMotor.setPower(-0.01);
                        stopping.set(false);
                    } else {
                        intakeMotor.setPower(1);
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        stopping.set(false);
                    }
                }).start();
            }

            // Check if the "A" button is pressed to spit the item back out
            if (gamepad1.a) {
                intakeMotor.setPower(1); // Spit the item back out
                sleep(500); // Run the intake motor in reverse for 0.5 seconds
                intakeMotor.setPower(0); // Stop the intake motor
            }

            /*

            // Get the color values from the sensor
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Calculate the ratios
            double total = red + green + blue;
            double redRatio = red / total;
            double greenRatio = green / total;
            double blueRatio = blue / total;


            // Check for colors
            if (redRatio > 0.4 && greenRatio < 0.3 && blueRatio < 0.3) {
                telemetry.addData("Color", "Red");
            } else if (redRatio > 0.3 && greenRatio > 0.3 && blueRatio < 0.3) {
                telemetry.addData("Color", "Yellow");
            } else if (blueRatio > 0.4 && redRatio < 0.3 && greenRatio < 0.3) {
                telemetry.addData("Color", "Blue");
            } else {
                telemetry.addData("Color", "Unknown");
            }

            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);
            telemetry.addData("Green", green);
            */

        }
    }
}
