package utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestHardwareMap {
    public DcMotorEx intaketest;
    public ColorSensor intakeColorSensor;

    public TestHardwareMap(HardwareMap testhardwareMap) {
        //Motors

        intaketest = testhardwareMap.get(DcMotorEx.class, "intaketest");

        intakeColorSensor = testhardwareMap.get(ColorSensor.class, "intakeColorSensor");

        intaketest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
