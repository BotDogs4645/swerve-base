package frc.bdlib.driver;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bdlib.driver.JoystickAxisAIO.LineFunctionType;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickButtonID;
import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickVariant;

public class ControllerAIO extends GenericHID {
    private JoystickVariant joystick_type;
    private HashMap<JoystickButtonID, JoystickButton> actual_buttons = new HashMap<JoystickButtonID, JoystickButton>();

    public ControllerAIO(final int port) {
        super(port);
        Optional<JoystickVariant> found = JoystickVariant.findJoy(DriverStation.getJoystickName(super.getPort()));
        if (found.isPresent()) {
            joystick_type = found.get();
            System.out.println("hello");
        } else {
            joystick_type = JoystickVariant.XBOX;
        }

        for (JoystickButtonID id: JoystickButtonID.values()) {
            actual_buttons.put(id, new JoystickButton(this, joystick_type.getButton(id)));
        }

    }

    public JoystickButton getJoystickButton(JoystickButtonID id) {
        return actual_buttons.get(id);
    }

    public ToggleBooleanSupplier getToggleBooleanSupplier(JoystickButtonID id, double debounce) {
        return new ToggleBooleanSupplier(actual_buttons.get(id), debounce);
    }
 
    public JoystickAxisAIO getAxis(JoystickAxisID id, double deadzone) {
        return new JoystickAxisAIO(this, id, deadzone);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id, LineFunctionType line_function, double deadzone) {
        return new JoystickAxisAIO(this, id, line_function, deadzone);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id, LineFunctionType line_function) {
        return new JoystickAxisAIO(this, id, line_function);
    }

    public JoystickAxisAIO getAxis(JoystickAxisID id) {
        return new JoystickAxisAIO(this, id);
    }

    public JoystickVariant getVariant() {
        return joystick_type;
    }

    public boolean getRumble() {
        return joystick_type.getCanRumble();
    }
}
