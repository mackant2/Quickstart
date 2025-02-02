package components;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import utils.Robot;

public class Drivetrain {
  Gamepad driverController;
  Set<Double> velocities = new HashSet<>();
  double rbVelocity;
  double rfVelocity;
  double lbVelocity;
  double lfVelocity;
  DcMotorEx rightBack, rightFront, leftBack, leftFront;
  double rotationFactor = 0.6;
  IMU imu;
  Orientation angles;
  Robot robot;
  float degree_Zero = 0;
  final double SPEED_MULT = 1.5;
  double drive_mult = 1;
  double rot_mult = 1;

  SparkFunOTOS myOtos;


  public void resetOrientation() {
      degree_Zero = angles.firstAngle;
  }

  public Drivetrain(Robot robot) {

    this.robot = robot;
    this.driverController = robot.opMode.gamepad1;
    rightBack = robot.parsedHardwareMap.backRight;
    rightFront = robot.parsedHardwareMap.frontRight;
    leftBack = robot.parsedHardwareMap.backLeft;
    leftFront = robot.parsedHardwareMap.frontLeft;

    myOtos = robot.parsedHardwareMap.myOtos;

    imu = robot.parsedHardwareMap.imu;
  }

  public void ToggleHalfSpeed() {
    drive_mult = drive_mult == 1 ? 0.2 : 1;
    rot_mult = rot_mult == 1 ? 0.4 : 1;
  }


  public void Update() {
    angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double  theta = angles.firstAngle - degree_Zero;
    if (theta > 180) {
      theta -= 360;
    }
    else if (theta < -180){
      theta += 360;
    }

    SparkFunOTOS.Pose2D pos = myOtos.getPosition();

    robot.opMode.telemetry.addData("X coordinate", pos.x);
    robot.opMode.telemetry.addData("Y coordinate", pos.y);
    robot.opMode.telemetry.addData("Heading angle", pos.h);

    robot.opMode.telemetry.addData("angle", theta);

    double x1 = driverController.left_stick_x;
    double y1 = driverController.left_stick_y * drive_mult;
    double r = driverController.right_stick_x * rotationFactor * rot_mult;
    double x = x1 * (Math.cos(Math.toRadians(theta))) - y1 * (Math.sin(Math.toRadians(theta)));
    double y = x1 * (Math.sin(Math.toRadians(theta))) + y1 * (Math.cos(Math.toRadians(theta)));
    rbVelocity = x - y - r;
    rfVelocity = -x - y - r;
    lbVelocity = -x - y + r;
    lfVelocity = x - y + r;
    velocities.add(rbVelocity * SPEED_MULT); //right back
    velocities.add(rfVelocity * SPEED_MULT); //right front
    velocities.add(lbVelocity * SPEED_MULT); //left back
    velocities.add(lfVelocity * SPEED_MULT); //left front
    double fastestMotorSpeed = Math.abs(Collections.max(velocities));
    velocities.clear();
    if (fastestMotorSpeed > SPEED_MULT) {
      rbVelocity /= fastestMotorSpeed;
      rfVelocity /= fastestMotorSpeed;
      lbVelocity /= fastestMotorSpeed;
      lfVelocity /= fastestMotorSpeed;
    }
    rightBack.setPower(rbVelocity);
    rightFront.setPower(rfVelocity);
    leftBack.setPower(lbVelocity);
    leftFront.setPower(lfVelocity);
  }
}
