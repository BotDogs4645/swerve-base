package frc.bdlib.custom_talon;
/**
* Unit control modes for use within the <code>TalonFXW</code> class. 
* @author David Muchow
* @version 1.0a
*/

public enum SensorUnits {

    /**
    * Unit output is in CTRE units.
    * There are no processing changes to the outputs of the motor.
    */
    CTRE,

    /**
     * <br></br>
     * Unit output is in Metric units.
     * Units used are: 
     * Meters, seconds, kilograms, watt, newton-meter, etc...
     */
    METRIC,

    /**
     * Unit output is in Imperial units.
     * Units used are: 
     * Feet, seconds, pounds, horsepower, torque, etc...
     */
    IMPERIAL
}
