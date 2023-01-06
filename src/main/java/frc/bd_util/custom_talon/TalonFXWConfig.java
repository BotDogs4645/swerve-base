package frc.bd_util.custom_talon;

/**
 * Settings for a TalonFXW<p>
 * If you are working with a Falcon 500 that does not have an attached object, this wrapper is unusuable.
 * @author David Muchow
 * @version 1.0b
*/

public class TalonFXWConfig {
    // Base Imperial
    private double gearing;
    private double diameter;

    /**
     * Settings for Talons directly attached to a wheel-like object. Gearing is assumed to be one.
     * @param diameter diameter of the wheel-like object attached to the motor in inches.
    */
    public TalonFXWConfig(double diameter) {
        this.diameter = diameter / 12;
        this.gearing = 1;
    }

    /**
     * Settings for Talons with gearing and attached to a wheel-like object.
     * @param gearing gear ratio of the connection to the 
     * @param diameter diameter of the wheel-like object attached to the motor in inches.
    */
    public TalonFXWConfig(double gearing, double diameter) {
        this.gearing = gearing;
        this.diameter = diameter / 12;
    }

    /**
     * @return The gearing of this motor.
    */
    public double getGearing() {
        return gearing;
    }

    /**
     * @return The diameter of the object attached to the mtoro.
    */
    public double getDiameter() {
        return diameter;
    }
    
}
