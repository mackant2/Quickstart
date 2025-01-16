package utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class EnhancedColorSensor {
    public enum Color {
        Any,
        Blue,
        Red,
        Yellow
    }
    static float minSaturation = 0.4f;
    static float minValue = 0.4f;
    static float[] hsvValues = new float[3];

    public static boolean CheckSensor(ColorSensor colorSensor, DistanceSensor distanceSensor, Color color) {
        // Get the color values from the sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        android.graphics.Color.RGBToHSV(red, green, blue, hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        if (saturation >= minSaturation && value >= minValue && distanceSensor.getDistance(DistanceUnit.CM) < 7) {
            //check if color matches any sample colors, or else white
            if (hue >= 0 && hue < 65) { //red
                return color == Color.Red || color == Color.Any;
            }
            else if (hue >= 65 && hue < 100) { //yellow
                return color == Color.Yellow || color == Color.Any;
            }
            else if (hue >= 165 && hue < 240) { //blue
                return color == Color.Blue || color == Color.Any;
            }
        }

        return false;
    }
}
