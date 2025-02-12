package utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import components.IntakeTest;
//import org.firstinspires.ftc.teamcode.main.components.IntakeTest;
//import org.firstinspires.ftc.teamcode.components.util.Logger;

public class RobotTest {

    public IntakeTest intaketest;
    public Logger logger;

    public OpMode opMode;
    public TestHardwareMap testHardwareMap;

    public void Initialize() {
        //initialize four bar
        //flip intake up and bring in
        intaketest.Initialize();
    }

    public RobotTest(OpMode opMode, TestHardwareMap testHardwareMap, boolean isTeleop) {
        this.testHardwareMap = testHardwareMap;
        this.opMode = opMode;

        intaketest = new IntakeTest(testHardwareMap, this);


        if (isTeleop) {
            Initialize();
        }
    }

    public void update() {
        intaketest.update();
    }
}
