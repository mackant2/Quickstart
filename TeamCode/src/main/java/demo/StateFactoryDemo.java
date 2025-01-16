package demo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "[DEMO] StateFactory", group = "demo")
public class StateFactoryDemo extends LinearOpMode {
    enum States {
        Extending,
        Dropping,
        Retracting
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*Logger logger = new Logger();
        logger.Initialize(telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Arm arm = new Arm(drive, this, () -> {});
        Intake intake = new Intake(drive, this, logger, () -> {});

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Extending)
                .onEnter(() -> arm.GoToHeight(Arm.Height.UPPER_BUCKET))
                .transition(() -> !arm.IsMoving())
                .state(States.Dropping)
                .onEnter(() -> intake.SetRunning(true))
                .transition(() -> !intake.IsRunning())
                .state(States.Retracting)
                .onEnter(() -> arm.GoToHeight(Arm.Height.DOWN))
                .build();

        Telemetry.Item stateItem = telemetry.addData("State", stateMachine.getState());

        waitForStart();

        if (isStopRequested()) return;

        stateMachine.start();
        while (opModeIsActive()) {
            //loop
            stateMachine.update();
            stateItem.setValue(stateMachine.getState());
            telemetry.update();
        }*/
    }
}