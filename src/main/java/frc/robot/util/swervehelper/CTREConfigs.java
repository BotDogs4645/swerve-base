package frc.robot.util.swervehelper;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.util.Units;
import frc.bdlib.custom_talon.TalonFXW;
import frc.robot.util.swervehelper.SwerveSettings.SwerveDriveTrain;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public TalonFXW.FXWConfig angleFXWConfig;
    public TalonFXW.FXWConfig driveFXWConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        StatorCurrentLimitConfiguration angleSupplyLimit = new StatorCurrentLimitConfiguration(
            SwerveDriveTrain.angleEnableCurrentLimit, 
            SwerveDriveTrain.angleContinuousCurrentLimit, 
            SwerveDriveTrain.anglePeakCurrentLimit, 
            SwerveDriveTrain.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = SwerveDriveTrain.angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveDriveTrain.angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveDriveTrain.angleKD;
        swerveAngleFXConfig.slot0.kF = SwerveDriveTrain.angleKF;
        swerveAngleFXConfig.statorCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        angleFXWConfig = new TalonFXW.FXWConfig(SwerveDriveTrain.angleGearRatio, Units.metersToInches(SwerveDriveTrain.wheelDiameter));
        driveFXWConfig = new TalonFXW.FXWConfig(SwerveDriveTrain.driveGearRatio, Units.metersToInches(SwerveDriveTrain.wheelDiameter));

        /* Swerve Drive Motor Configuration */
        StatorCurrentLimitConfiguration driveSupplyLimit = new StatorCurrentLimitConfiguration(
            SwerveDriveTrain.driveEnableCurrentLimit, 
            SwerveDriveTrain.driveContinuousCurrentLimit, 
            SwerveDriveTrain.drivePeakCurrentLimit, 
            SwerveDriveTrain.drivePeakCurrentDuration);

        

        swerveDriveFXConfig.slot0.kP = SwerveDriveTrain.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveDriveTrain.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveDriveTrain.driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveDriveTrain.driveKF;        
        swerveDriveFXConfig.statorCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = SwerveDriveTrain.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveDriveTrain.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveDriveTrain.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}