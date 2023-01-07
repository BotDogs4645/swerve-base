package frc.bdlib.driver;

import java.util.function.BooleanSupplier;

import frc.bdlib.misc.BDConstants.JoystickConstants.JoystickAxisID;

public class JoystickAxisAIO {
    public enum LineFunctionType {
        Linear {
            @Override
            public double multiplier(double in) {
                return in;
            }
        },
        Aggressive {
            @Override
            public double multiplier(double in) {
                return ((2.0/3.0) * in) + ((1.0/3.0) * Math.pow(in, 3));
            }
        },
        Intermediate {
            @Override
            public double multiplier(double in) {
                return ((1.0/2.0) * in) + ((1.0/2.0) * Math.pow(in, 3));
            }
        },
        Gentle {
            @Override
            public double multiplier(double in) {
                return ((1.0/3.0) * in) + ((2.0/3.0) * Math.pow(in, 3));
            }
        },
        Cubic {
            @Override
            public double multiplier(double in) {
                return Math.pow(in, 3);
            }
        };

        abstract public double multiplier(double in);
    }

    ControllerAIO controller;
    JoystickAxisID permanent_id;
    LineFunctionType type;
    double deadzone;

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id) {
        this.controller = controller;
        this.permanent_id = permanent_id;
        this.type = LineFunctionType.Intermediate;
        this.deadzone = 0.0;
    }

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id, LineFunctionType type) {
        this.controller = controller;
        this.permanent_id = permanent_id;
        this.type = type;
        this.deadzone = 0.0;
    }

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id, double deadzone) {
        this.controller = controller;
        this.permanent_id = permanent_id;
        this.type = LineFunctionType.Intermediate;
        this.deadzone = deadzone;
    }

    public JoystickAxisAIO(ControllerAIO controller, JoystickAxisID permanent_id, LineFunctionType type, double deadzone) {
        this.controller = controller;
        this.permanent_id = permanent_id;
        this.type = type;
        this.deadzone = deadzone;
    }

    public double getValue() {
        double in = controller.getRawAxis(controller.getVariant().getAxis(permanent_id));

        double value = (Math.abs(in) < deadzone) ? 0 : in;
        value = type.multiplier(in);
        return value;
    }

    public BooleanSupplier axisHigherThan(double value) {
        return () -> this.getValue() > value;
    }
}
