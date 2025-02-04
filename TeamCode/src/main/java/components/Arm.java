package components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import utils.Robot;

public class Arm {
    public enum ArmState {
        DriverControlled,
        Transferring
    }
    public static class FourBarPosition {
        public static final double Bucket = .68;
        public static final double SpecimenHang = 1;
        public static final double StraightBack = 0.21;
        public static final double StraightUp = 0.54;
    }
    public static class WristPosition {
        public static final double Transfer = .86;
        public static final double Specimen = 0.69;//.58
        public static final double Straight = 0.5;
        public static final double SpecPreset = .35;
    }
    public static class Height {
        public static final int LOWER_BUCKET = 2400;
        public static final int UPPER_BUCKET = 3650;
        public static final int UPPER_BAR = 1800;
        public static final int DOWN = 0;
        public static final int Transfer = 100;
        public static int WallPickup = 157;
        public static final int SpecArmPreset = 650;
    }
    public static class ClawPosition {
        public static final double Open = 0.9;
        public static final double Closed = 0;
    }
    public ArmState state = ArmState.DriverControlled;
    Telemetry telemetry;
    Gamepad assistantController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    final double MAX_FOURBAR_SPEED = 0.05;
    final float MAX_HEIGHT = Height.UPPER_BUCKET;
    final int LIFT_MAX_DIFF = 400;
    Robot robot;
    boolean didStateAction = false;

    public void RotateFourBar(double position) {
        leftFourBar.setPosition(1 - position);
        rightFourBar.setPosition(position);
    }

    public void RotateWrist(double degrees) {
        wrist.setPosition(degrees);
        //wrist.setPosition(degrees / 180f + 0.5f);
    }

    double GetWristDegrees() {
        return wrist.getPosition() / 180 + 0.5;
    }

    public void PrepareToGrabSpecimen() {
        RotateFourBar(FourBarPosition.StraightBack);
        wrist.setPosition(WristPosition.Straight);
        //claw.setPosition(ClawPosition.Open);
        claw.setPosition(ClawPosition.Open); //TODO: go back to open claw position after new claw is printed and fourbar passthrough with open claw is allowed
        GoToHeight(Height.WallPickup);
    }

    public void PresetSpecScore() {
        wrist.setPosition(WristPosition.SpecPreset);
        RotateFourBar(FourBarPosition.StraightUp);
        GoToHeight(Height.SpecArmPreset);
    }

    public void TransferPreset(){
        RotateFourBar(FourBarPosition.StraightBack);
        RotateWrist(WristPosition.Transfer);
        GoToHeight(Height.Transfer);
       // SetClawPosition(ClawPosition.Closed);
    }

    public void PrepareToDepositSpecimen() {
        GoToHeight(Height.UPPER_BAR);
        RotateFourBar(FourBarPosition.StraightBack);
        wrist.setPosition(WristPosition.Specimen);
    }

    public void PrepareToTransfer() {
        GoToHeight(Height.DOWN);
        RotateFourBar(FourBarPosition.StraightBack);
        RotateWrist(WristPosition.Transfer);
        claw.setPosition(ClawPosition.Open);
    }

    public void UpdateWallPickupHeight() {
        robot.logger.Log("WALL PICKUP HEIGHT: " + liftLeft.getTargetPosition());
        Height.WallPickup = liftLeft.getTargetPosition();
    }

    public Arm(Robot robot) {
        this.robot = robot;
        this.assistantController = robot.opMode.gamepad2;
        this.telemetry = robot.opMode.telemetry;

        //assign motors
        liftLeft = robot.parsedHardwareMap.liftLeft;
        liftRight = robot.parsedHardwareMap.liftRight;
        //assign servos
        //four bar - 0 is out, 1 is transfer position
        leftFourBar = robot.parsedHardwareMap.leftFourBar;
        rightFourBar = robot.parsedHardwareMap.rightFourBar;
        wrist = robot.parsedHardwareMap.wrist;
        claw = robot.parsedHardwareMap.claw;
    }

    public void Initialize() {
        SetClawPosition(Arm.ClawPosition.Closed);
        robot.delaySystem.CreateDelay(1500, this::Reset);
    }

    public void ToggleClaw() {
      //  if (state == ArmState.DriverControlled) {
            claw.setPosition(claw.getPosition() == ClawPosition.Open ? ClawPosition.Closed : ClawPosition.Open);
      //  }
     //   claw.setPosition(ClawPosition.Closed);
    }

    public void SetClawPosition(double newPosition) {
        claw.setPosition(newPosition);
    }

    public void GoToHeight(int height) {
        liftLeft.setTargetPosition(height);
        liftRight.setTargetPosition(height);
    }

    public void AdjustLiftHeight(int change) {
        GoToHeight(liftLeft.getCurrentPosition() + change);
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        switch (state) {
            case DriverControlled:
                double power = -assistantController.left_stick_y;
                if (power != 0) {
                    if (assistantController.left_bumper) {
                        power *= 0.2;
                    }
                    GoToHeight((int)clamp((float)(liftLeft.getCurrentPosition() + 1 + Math.floor(power * LIFT_MAX_DIFF)), 0, MAX_HEIGHT));
                }

                double fourBarPosition = rightFourBar.getPosition();
                double change = -assistantController.right_stick_y * MAX_FOURBAR_SPEED;
                if (assistantController.right_bumper) {
                    change *= 0.5;
                }
                //Math.clamp causes crash here, so using custom method
                double clamped = clamp((float)(fourBarPosition + change * 0.5), (float)FourBarPosition.StraightBack, 1);
                if (change != 0) {
                    RotateFourBar(clamped);
                }

                wrist.setPosition(clamp((float)(wrist.getPosition() + (assistantController.left_trigger - assistantController.right_trigger) * 0.02), 0, 1));
            break;
            case Transferring:
                if (robot.parsedHardwareMap.extender.getCurrentPosition() > Intake.ExtenderPosition.IN + 30) {
                    RotateFourBar(FourBarPosition.StraightBack);
                    RotateWrist(WristPosition.Transfer);
                    SetClawPosition(ClawPosition.Closed);
                    GoToHeight(Height.Transfer + 500);
                }
                else if (!didStateAction) {
                    didStateAction = true;
                    robot.delaySystem.CreateDelay(1000, () -> {
                        SetClawPosition(ClawPosition.Open);
                        robot.delaySystem.CreateDelay(500, () -> {
                            GoToHeight(Height.Transfer);
                            robot.delaySystem.CreateDelay(1000, () -> {
                                SetClawPosition(ClawPosition.Closed);
                                robot.delaySystem.CreateDelay(500, () -> {
                                    GoToHeight(Height.Transfer + 1000);
                                    robot.delaySystem.CreateDelay(500, () -> {
                                        didStateAction = false;
                                        state = ArmState.DriverControlled;
                                    });
                                });
                            });
                        });
                    });
                }
            break;
        }

        telemetry.addData("Arm State", state);
        robot.opMode.telemetry.addData("Arm Voltage (MILLIAMPS)", liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Lift Position", liftLeft.getCurrentPosition());
        telemetry.addData("Lift Target", liftLeft.getTargetPosition());
        telemetry.addData("Four Bar Position", leftFourBar.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
    }

    public void HangSpecimen() {
        GoToHeight(2700);
    }

    public void ScoreSample() {
        GoToHeight(Height.UPPER_BUCKET);
        RotateFourBar(FourBarPosition.Bucket);
        wrist.setPosition(WristPosition.Straight);
    }

    public void Reset() {
        GoToHeight(Height.DOWN);
        wrist.setPosition(0.1);
        RotateFourBar(FourBarPosition.StraightUp);
        claw.setPosition(ClawPosition.Closed);
    }

    public void RunPreset(Preset preset) {
        GoToHeight(preset.liftPosition);
        RotateFourBar(preset.fourBarPosition);
        RotateWrist(preset.wristPosition);
    }

    public static class Presets {
        public Preset ResetUp = new Preset(Height.DOWN, FourBarPosition.StraightUp, 0.1);
    }

    static class Preset {
        int liftPosition;
        double fourBarPosition;
        double wristPosition;

        public Preset(int liftPosition, double fourBarPosition, double wristPosition) {
            this.liftPosition = liftPosition;
            this.fourBarPosition = fourBarPosition;
            this.wristPosition = wristPosition;
        }
    }
}