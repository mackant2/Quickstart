package utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import components.Arm;
import components.Drivetrain;
import components.Intake;

public class Robot {
    public Arm arm;
    public Drivetrain drivetrain;
    public Intake intake;
    public Logger logger;
    public OpMode opMode;
    Telemetry telemetry;
    public ParsedHardwareMap parsedHardwareMap;
    public PressEventSystem pressEventSystem;
    public DelaySystem delaySystem = new DelaySystem();
    boolean isTeleop;

    void InitializeAll() {
        //initialize four bar
        arm.runPreset(Arm.Presets.RESET);
        //flip intake up and bring in
        intake.reset();
    }

    public Robot(OpMode opMode, HardwareMap hardwareMap, boolean isTeleop) {
        this.opMode = opMode;
        this.parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        this.isTeleop = isTeleop;
        this.telemetry = opMode.telemetry;
        this.pressEventSystem = new PressEventSystem(telemetry);
        arm = new Arm(this);
        drivetrain = new Drivetrain(this);
        logger = new Logger(opMode.telemetry);
        intake = new Intake(this);

        if (!isTeleop) {
            InitializeAll();
        }
    }

    public void update() {
        arm.internalUpdate();
        intake.internalUpdate();

        if (isTeleop) {
            arm.update();
            drivetrain.update();
            intake.update();
        }

        delaySystem.update();
        pressEventSystem.update();
    }
}
