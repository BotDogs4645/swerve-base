package frc.bd_util.driver;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleBooleanSupplier {
    //private JoystickButton orginal_button;
    private Trigger updated_button;
    private boolean actual_value;

    public ToggleBooleanSupplier(JoystickButton button, double debounce) {
        //this.orginal_button = button;
        this.updated_button = button.debounce(debounce, DebounceType.kFalling);
        this.actual_value = updated_button.getAsBoolean();

        updated_button.onTrue(
            new InstantCommand(() -> {actual_value = !actual_value;})
        );
    }

    public boolean getValue() {
        return actual_value;
    }
}
