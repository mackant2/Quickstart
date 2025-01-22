package utils;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ParsedHardwareMap {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx liftLeft, liftRight, extender, intake;
    public Servo claw, leftFourBar, rightFourBar, wrist, flipDown, gate;
    public RevBlinkinLedDriver display;
    public ColorSensor leftColorSensor, rightColorSensor, transferSensor;
    public DistanceSensor leftDistanceSensor, rightDistanceSensor, transferDistanceSensor;
    public TouchSensor liftLimiter, extenderLimiter;
    public IMU imu;
    public SparkFunOTOS myOtos;

    public ParsedHardwareMap(HardwareMap hardwareMap) {
        //Motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        extender = hardwareMap.get(DcMotorEx.class, "Extendo");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        claw = hardwareMap.get(Servo.class, "claw");
        leftFourBar = hardwareMap.get(Servo.class, "leftFourBar");
        rightFourBar = hardwareMap.get(Servo.class, "rightFourBar");
        wrist = hardwareMap.get(Servo.class, "wrist");
        flipDown = hardwareMap.get(Servo.class, "flipdown");
        gate = hardwareMap.get(Servo.class, "gate");

        //Sensors
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        transferSensor = hardwareMap.get(ColorSensor.class, "transferSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");
        transferDistanceSensor = hardwareMap.get(DistanceSensor .class, "transferSensor");
        liftLimiter = hardwareMap.get(TouchSensor.class, "liftLimiter");
        extenderLimiter = hardwareMap.get(TouchSensor.class, "extenderLimiter");

        //LEDs
        display = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //Configure Drivetrain
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Configure Lift
        liftLeft.setTargetPosition(liftLeft.getCurrentPosition());
        liftRight.setTargetPosition(liftRight.getCurrentPosition());
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFourBar.setDirection(Servo.Direction.FORWARD);
        rightFourBar.setDirection(Servo.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setPower(1);
        liftRight.setPower(1);

        //Configure Intake
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setTargetPosition(extender.getCurrentPosition());
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setPower(1);

        imu = hardwareMap.get(IMU.class, "imu");
    }
}
