package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import utils.ParsedHardwareMap;


@TeleOp (group = "tuning", name="[TUNING] Drivetrain")
public class Drivetrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        DcMotorEx frontLeft, frontRight, backLeft, backRight;
        frontLeft = parsedHardwareMap.frontLeft;
        frontRight = parsedHardwareMap.frontRight;
        backLeft = parsedHardwareMap.backLeft;
        backRight = parsedHardwareMap.backRight;

        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);

        while (!isStopRequested()) {
            telemetry.addData("Left front(par0)", frontLeft.getCurrentPosition());
            telemetry.addData("Back right(perp)", backRight.getCurrentPosition());
            telemetry.addData("Front right (par1)", frontRight.getCurrentPosition());
            telemetry.update();
        }
    }
}