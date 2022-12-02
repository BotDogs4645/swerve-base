package frc.bd_util.custom_talon;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import frc.bd_util.*;

/**
* Custom implementation of the {@code WPI_TalonFX} class.
* @author David Muchow
* @version 1.0.0
*/

public class TalonFXW extends WPI_TalonFX implements BDUpdatable {
    // Units. Changes how the outputs of the motor are converted when we use get methods.
    private SensorUnits sensor_units = SensorUnits.CTRE;
    private TalonFXWConfig configuration;
    private int id;

    /**
     * For Talon controllers on the roboRIO CAN bus.
     * @param can_id device id of the Talon controller
     * @param configuration the {@link TalonFXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
    */
    public TalonFXW(int can_id, TalonFXWConfig configuration) {
        super(can_id);
        this.configuration = configuration;
        this.id = can_id;

        BDManager.getInstance().register(this);
    }

    /**
     * For Talon controllers on a CANivore.
     * @param can_id device ID of the Talon controller
     * @param can_bus id of the CAN bus, most likely "canivore" or "swerve"
     * @param configuration the {@link TalonFXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
    */
    public TalonFXW(int can_id, String can_bus, TalonFXWConfig configuration) {
        super(can_id, can_bus);
        this.configuration = configuration;
        this.id = can_id;

        BDManager.getInstance().register(this);
    }

    /**
     * For Talon controllers on the roboRIO CAN bus with desired unit conversions.
     * @param can_id device ID of the Talon controller
     * @param sensor_units the decided units via the {@link SensorUnits} enum class. 
     * @param configuration the {@link TalonFXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
    */
    public TalonFXW(int can_id, SensorUnits sensor_units, TalonFXWConfig configuration) {
        super(can_id);
        this.sensor_units = sensor_units;
        this.configuration = configuration;
        this.id = can_id;

        BDManager.getInstance().register(this);
    }

    /**
     * For Talon controllers on the CANivore CAN bus with desired unit conversions.
     * @param can_id device ID of the Talon controller
     * @param can_bus id of the CAN bus, most likely "canivore" or "swerve"
     * @param sensor_units the decided units via the {@link SensorUnits} enum class. 
     * @param configuration the {@link TalonFXWConfig} configuration that signifies the gearing ratio and attached object diameter of the motor.
    */
    public TalonFXW(int can_id, String can_bus, SensorUnits sensor_units, TalonFXWConfig configuration) {
        super(can_id, can_bus);
        this.sensor_units = sensor_units;
        this.id = can_id;

        BDManager.getInstance().register(this);
    }

    /**
     * @return How many rotations the shaft has made.
    */
    public double getShaftRotations() {
        return super.getSelectedSensorPosition() / 2048;
    }
    
    /**
     * @return How many full rotations the shaft makes every 100ms
    */
    public double getShaftVelocity() {
        return super.getSelectedSensorVelocity() / 2048;
    }

    /**
     * @return How many rotations the object, through gearing, makes per second.
    */
    public double getObjectRotationsPerSecond() {
        return (getShaftVelocity() * 10) / configuration.getGearing();
    }

    /**
     * @return How many rotations the object, through gearing, makes per minute.
    */
    public double getObjectRotationsPerMinute() {
        return getObjectRotationsPerSecond() * 60;
    }

    public double getObjectTotalRotations() {
        return (getShaftRotations() / configuration.getGearing());
    }

    /**
     * Depending on the sensor mode, you will either get:<p>
     * <b>Metric:</b> Total wheel rotations made in meters
     * <b>Imperial:</b> Total wheel rotations in feet
     * <b>CTRE:</b> Total wheel rotations in well.. rotations.
     * @return The total wheel rotations in selected unit.
     */
    public double getObjectTotalDistanceTravelled() {
        if (sensor_units == SensorUnits.METRIC) {
            double wheelRotations = (getShaftRotations() / configuration.getGearing());
            return Units.feetToMeters(wheelRotations * configuration.getDiameter() * Math.PI);
        }
        if (sensor_units == SensorUnits.IMPERIAL) {
            double wheelRotations = (getShaftRotations() / configuration.getGearing());
            return wheelRotations * configuration.getDiameter() * Math.PI;
        }

        // returns how many rotations the object has made.
        return getObjectTotalRotations();
    }

    /**
     * Depending on the sensor mode, you will either get:<p>
     * <b>Metric:</b> Current wheel rotations in meters per second
     * <b>Imperial:</b> Current wheel rotations in feet per second
     * <b>CTRE:</b> Current wheel rotations in rotations per second.
     * @return The wheel velocity in selected unit.
     */
    public double getObjectConvertedVelocity() {
        if (sensor_units == SensorUnits.METRIC) {
            double wheelVelocityInRotations = getObjectRotationsPerSecond();
            return Units.feetToMeters(wheelVelocityInRotations * configuration.getDiameter() * Math.PI); 
        }
        if (sensor_units == SensorUnits.IMPERIAL) {
            double wheelVelocityInRotations = getObjectRotationsPerSecond();
            return wheelVelocityInRotations * configuration.getDiameter() * Math.PI; 
        }

        // returns how many rotations the object has made.
        return getObjectRotationsPerSecond();
    }

    /**
     * @return The configuration of this particular motor.
    */
    public TalonFXWConfig getConfig() {
        return configuration;
    }

    /** <i>Warning:</i> might be error prone.
     * @return Set the position of the motor to zero.
    */
    public void zero() {
        super.setSelectedSensorPosition(0, 0, 0);
    }

    public void update() {}

    public void updateStatus() {}

    public String getID() {
        return "TalonFXW " + id;
    }

    public String getStatus() {
        Faults current_faults = new Faults();
        super.getFaults(current_faults);

        return current_faults.hasAnyFault() ? current_faults.toString() : "OK";
    }
}