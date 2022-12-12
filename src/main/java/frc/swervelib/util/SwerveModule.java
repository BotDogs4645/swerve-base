package frc.swervelib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.bd_util.custom_talon.SensorUnits;
import frc.bd_util.custom_talon.TalonFXW;
import frc.bd_util.pidtuner.PIDTunerTalon;
import frc.robot.Robot;
import frc.swervelib.math.Conversions;
import frc.swervelib.util.SwerveConstants.Swerve.testing_type;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFXW mAngleMotor;
    private TalonFXW mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private ShuffleboardLayout layout;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.Swerve.driveKS, SwerveConstants.Swerve.driveKV, SwerveConstants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, ShuffleboardTab sub_tab) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFXW(moduleConstants.angleMotorID, SensorUnits.METRIC, Robot.ctreConfigs.angleFXWConfig);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFXW(moduleConstants.driveMotorID, SensorUnits.METRIC, Robot.ctreConfigs.driveFXWConfig);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();

        layout = sub_tab.getLayout("mod " + moduleNumber, BuiltInLayouts.kList);

        layout.addDouble("Cancoder", () -> getCanCoder().getDegrees());
        layout.addDouble("Integrated", () -> getState().angle.getDegrees());
        layout.addDouble("Velocity", () -> getState().speedMetersPerSecond);

        if (moduleConstants.type == testing_type.DRIVE) {
            new PIDTunerTalon(mDriveMotor, sub_tab);
        }

        if (moduleConstants.type == testing_type.STEER) {
            new PIDTunerTalon(mAngleMotor, sub_tab);
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, SwerveConstants.Swerve.angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, SwerveConstants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveConstants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(SwerveConstants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    public void reapplyConfig() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveConstants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(SwerveConstants.Swerve.angleNeutralMode);

        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveConstants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(SwerveConstants.Swerve.driveNeutralMode);
    }

    private void configDriveMotor() {        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveConstants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(SwerveConstants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getSwervePosition() {
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.Swerve.angleGearRatio));
        return new SwerveModulePosition(mDriveMotor.getObjectTotalDistanceTraveled(), angle);
    }

}