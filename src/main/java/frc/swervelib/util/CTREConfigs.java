package frc.swervelib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.util.Units;
import frc.bd_util.custom_talon.TalonFXWConfig;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public TalonFXWConfig angleFXWConfig;
    public TalonFXWConfig driveFXWConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveSettings.Swerve.angleEnableCurrentLimit, 
            SwerveSettings.Swerve.angleContinuousCurrentLimit, 
            SwerveSettings.Swerve.anglePeakCurrentLimit, 
            SwerveSettings.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = SwerveSettings.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveSettings.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveSettings.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = SwerveSettings.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        angleFXWConfig = new TalonFXWConfig(SwerveSettings.Swerve.angleGearRatio, Units.metersToInches(SwerveSettings.Swerve.wheelDiameter));
        driveFXWConfig = new TalonFXWConfig(SwerveSettings.Swerve.driveGearRatio, Units.metersToInches(SwerveSettings.Swerve.wheelDiameter));

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveSettings.Swerve.driveEnableCurrentLimit, 
            SwerveSettings.Swerve.driveContinuousCurrentLimit, 
            SwerveSettings.Swerve.drivePeakCurrentLimit, 
            SwerveSettings.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = SwerveSettings.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveSettings.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveSettings.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveSettings.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = SwerveSettings.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveSettings.Swerve.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveSettings.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}