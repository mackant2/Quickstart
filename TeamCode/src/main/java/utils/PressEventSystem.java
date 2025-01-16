package utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PressEventSystem {
    private final Telemetry telemetry;
    private final List<Listener> listeners = new ArrayList<>();
    private final Map<String, Field> buttons = new HashMap<>();

    class Listener {
        public final Gamepad controller;
        public final Field button;
        public final PressCallback callback;
        public boolean isPressed;
        public Listener(Gamepad controller, String button, PressCallback callback) {
            this.controller = controller;
            this.button = buttons.get(button);
            this.callback = callback;
        }
    }

    public PressEventSystem(Telemetry telemetry) {
        this.telemetry = telemetry;
        //Cache buttons during init for faster loading
        for (Field field : Gamepad.class.getDeclaredFields()) {
            if (field.getType() == boolean.class) {
                buttons.put(field.getName(), field);
            }
        }
    }

    public interface PressCallback {
        void fire();
    }

    public void Update() {
        //Loop through listeners
        for (Listener listener : listeners) {
            try {
                boolean pressed = listener.button.getBoolean(listener.controller);
                if (pressed != listener.isPressed) {
                    listener.isPressed = pressed;
                    if (pressed) {
                        listener.callback.fire();
                    }
                }
            }
            catch (IllegalAccessException error) {
                telemetry.addData("PressEventSystem Error:", error.getMessage());
                telemetry.update();
            }
        }
    }

    public void AddListener(Gamepad controller, String button, PressCallback callback) {
        listeners.add(new Listener(controller, button, callback));
    }
}
