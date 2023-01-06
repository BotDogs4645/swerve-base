package frc.bd_util.driver;

import frc.bd_util.misc.BDConstants.JoystickConstants.JoystickAxisID;

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
                return ((2/3) * in) + ((1/3) * Math.pow(in, 3));
            }
        },
        Intermediate {
            @Override
            public double multiplier(double in) {
                return ((1/2) * in) + ((1/2) * Math.pow(in, 3));
            }
        },
        Gentle {
            @Override
            public double multiplier(double in) {
                return ((1/3) * in) + ((2/3) * Math.pow(in, 3));
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
}
