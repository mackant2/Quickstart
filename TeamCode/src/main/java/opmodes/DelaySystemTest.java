package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import utils.DelaySystem;

@TeleOp
public class DelaySystemTest extends OpMode {
    DelaySystem delaySystem = new DelaySystem();
    
    @Override
    public void init() {
        delaySystem.createConditionalDelay(
                () -> gamepad1.a,
                this::requestOpModeStop,
                2000,
                this::requestOpModeStop
        );
    }

    @Override
    public void init_loop() {
        delaySystem.update();
    }

    @Override
    public void loop() {
        
    }
}
