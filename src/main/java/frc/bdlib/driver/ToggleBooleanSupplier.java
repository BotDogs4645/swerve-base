package frc.bdlib.driver;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ToggleBooleanSupplier {
    //private JoystickButton orginal_button;
    private boolean actual_value;
    private double timeLastPressed = System.currentTimeMillis();

    public ToggleBooleanSupplier(JoystickButton button, double debounce) {
        //this.orginal_button = button;
        this.actual_value = false;

        new RunCommand(() -> {
            if (button.getAsBoolean() == true && (System.currentTimeMillis() - timeLastPressed) / 1000 > debounce) {
                actual_value = !actual_value;
                timeLastPressed = System.currentTimeMillis();
                System.out.println(actual_value);
            }
        })
        .ignoringDisable(true)
        .schedule();
    }

    public boolean getValue() {
        return actual_value;
    }
}
