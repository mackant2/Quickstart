package utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import components.Arm;
import components.Drivetrain;
import components.Intake;

public class Robot {
    public Arm arm;
    public Drivetrain drivetrain;
    public Intake intake;
    public Logger logger;
    public OpMode opMode;
    public ParsedHardwareMap parsedHardwareMap;

    void InitializeAll() {
        //initialize four bar
        arm.Initialize();
        //flip intake up and bring in
        intake.Initialize();
    }

    public Robot(OpMode opMode, HardwareMap hardwareMap, boolean isTeleop) {
        this.parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        this.opMode = opMode;
        arm = new Arm(this);
        drivetrain = new Drivetrain(this);
        logger = new Logger(opMode.telemetry);
        intake = new Intake(this);

        if (isTeleop) {
            InitializeAll();
        }
        else {
            intake.Initialize();
        }
    }

    public void Update() {
        arm.Update();
        drivetrain.Update();
        intake.Update();
    }
}
