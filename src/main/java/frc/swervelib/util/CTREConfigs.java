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
            SwerveConstants.Swerve.angleEnableCurrentLimit, 
            SwerveConstants.Swerve.angleContinuousCurrentLimit, 
            SwerveConstants.Swerve.anglePeakCurrentLimit, 
            SwerveConstants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = SwerveConstants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveConstants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveConstants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = SwerveConstants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        angleFXWConfig = new TalonFXWConfig(SwerveConstants.Swerve.angleGearRatio, Units.metersToInches(SwerveConstants.Swerve.wheelDiameter));
        driveFXWConfig = new TalonFXWConfig(SwerveConstants.Swerve.driveGearRatio, Units.metersToInches(SwerveConstants.Swerve.wheelDiameter));

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.Swerve.driveEnableCurrentLimit, 
            SwerveConstants.Swerve.driveContinuousCurrentLimit, 
            SwerveConstants.Swerve.drivePeakCurrentLimit, 
            SwerveConstants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = SwerveConstants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveConstants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveConstants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveConstants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = SwerveConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveConstants.Swerve.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveConstants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}