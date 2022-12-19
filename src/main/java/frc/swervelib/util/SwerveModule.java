package frc.swervelib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.bd_util.BDManager;
import frc.bd_util.custom_talon.SensorUnits;
import frc.bd_util.custom_talon.TalonFXW;
import frc.bd_util.pidtuner.PIDTunerTalon;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.swervelib.math.Conversions;
import frc.swervelib.util.SwerveSettings.ShuffleboardConstants.BOARD_PLACEMENT;
import frc.swervelib.util.SwerveSettings.Swerve.TESTING_TYPE;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public String name;
    public int moduleNumber;
    private double angleOffset;
    public TalonFXW mAngleMotor;
    public TalonFXW mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private ShuffleboardLayout layout;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveSettings.Swerve.driveKS, SwerveSettings.Swerve.driveKV, SwerveSettings.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, ShuffleboardTab sub_tab) {
        this.moduleNumber = moduleNumber;
        name = moduleConstants.name;
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

        BOARD_PLACEMENT placement = BOARD_PLACEMENT.valueOf("TEMP" + moduleNumber);

        layout = sub_tab.getLayout("module " + moduleNumber, BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2))
        .withPosition(placement.getX(), placement.getY())
        .withSize(1, 2);

        layout.addDouble("Angle Temp", () -> (mAngleMotor.getTemperature() * (9.0/5.0)) + 32); // C -> F
        layout.addDouble("Drive Temp", () -> (mDriveMotor.getTemperature() * (9.0/5.0)) + 32);

        ShuffleboardTab tab = BDManager.getInstance().getInstanceManagerialTab();
        if (Constants.testing) {
            ShuffleboardLayout lay = tab.getLayout("module " + moduleNumber, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
            lay.addDouble("Cancoder", () -> getCanCoder().getDegrees());
            lay.addDouble("Integrated", () -> getState().angle.getDegrees());
        }

        if (moduleConstants.type == TESTING_TYPE.DRIVE) {
            new PIDTunerTalon(mDriveMotor, Shuffleboard.getTab("mod " + moduleNumber + " Drive PID Tuner"));
        }

        if (moduleConstants.type == TESTING_TYPE.STEER) {
            new PIDTunerTalon(mAngleMotor, Shuffleboard.getTab("mod " + moduleNumber + " Steer PID Tuner"));
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveSettings.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveSettings.Swerve.wheelCircumference, SwerveSettings.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveSettings.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, SwerveSettings.Swerve.angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, SwerveSettings.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveSettings.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(SwerveSettings.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    public void reapplyConfig() {
        TalonFXConfiguration configAngle = new TalonFXConfiguration();

        mAngleMotor.getAllConfigs(configAngle);

        configAngle.closedloopRamp = SwerveSettings.Swerve.closedLoopRamp;
        configAngle.openloopRamp = SwerveSettings.Swerve.openLoopRamp;

        mAngleMotor.setInverted(SwerveSettings.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(SwerveSettings.Swerve.angleNeutralMode);
        mAngleMotor.configAllSettings(configAngle);


        TalonFXConfiguration configDrive = new TalonFXConfiguration();
        
        mDriveMotor.getAllConfigs(configDrive);

        configDrive.closedloopRamp = SwerveSettings.Swerve.closedLoopRamp;
        configDrive.openloopRamp = SwerveSettings.Swerve.openLoopRamp;

        mDriveMotor.setInverted(SwerveSettings.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(SwerveSettings.Swerve.driveNeutralMode);
        mDriveMotor.configAllSettings(configDrive);
    }

    private void configDriveMotor() {        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveSettings.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(SwerveSettings.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), SwerveSettings.Swerve.wheelCircumference, SwerveSettings.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveSettings.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getSwervePosition() {
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveSettings.Swerve.angleGearRatio));
        return new SwerveModulePosition(mDriveMotor.getObjectTotalDistanceTraveled(), angle);
    }

}