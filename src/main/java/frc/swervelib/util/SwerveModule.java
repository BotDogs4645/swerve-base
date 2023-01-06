package frc.swervelib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.bd_util.custom_talon.SensorUnits;
import frc.bd_util.custom_talon.TalonFXW;
import frc.bd_util.misc.BDManager;
import frc.bd_util.pidtuner.PIDTunerTalon;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.swervelib.math.Conversions;
import frc.swervelib.util.SwerveSettings.ShuffleboardConstants.BOARD_PLACEMENT;
import frc.swervelib.util.SwerveSettings.SwerveDriveTrain.TestingType;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveSettings.SwerveDriveTrain.driveKS, SwerveSettings.SwerveDriveTrain.driveKV, SwerveSettings.SwerveDriveTrain.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, ShuffleboardTab sub_tab) {
        // gets our number, name and angle offset from Settings.
        // Settings gets passed through Swerve subsystem and then to here.
        this.moduleNumber = moduleNumber;
        name = moduleConstants.name;
        angleOffset = moduleConstants.angleOffset;
        
        // More config
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFXW(moduleConstants.angleMotorID, Robot.ctreConfigs.angleFXWConfig, SensorUnits.METRIC);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFXW(moduleConstants.driveMotorID, Robot.ctreConfigs.driveFXWConfig, SensorUnits.METRIC);
        configDriveMotor();

        // gets the last angle for jitter analysis
        lastAngle = getState().angle.getDegrees();

        // gets the placement of the board via the Shuffleboard placement enumerator
        BOARD_PLACEMENT placement = BOARD_PLACEMENT.valueOf("TEMP" + moduleNumber);

        // Placing stuff on the shuffleboard layout
        layout = sub_tab.getLayout("m" + moduleNumber, BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2))
        .withPosition(placement.getX(), placement.getY())
        .withSize(1, 3);

        layout.addDouble("Angle Temp", () -> mAngleMotor.getTemperature()); // C -> F
        layout.addDouble("Drive Temp", () -> mDriveMotor.getTemperature());

        // Place super specific fault analysis. Make sure to turn on testing to get these values.
        ShuffleboardTab tab = BDManager.getInstance().getInstanceManagerialTab();
        if (Constants.testing) {
            ShuffleboardLayout lay = tab.getLayout("module " + moduleNumber, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
            lay.addDouble("Cancoder", () -> getCanCoder().getDegrees());
            lay.addDouble("Integrated", () -> getState().angle.getDegrees());
        }

        // Testing PID values with this system. To enable, go to swerve settings.
        if (moduleConstants.type == TestingType.DRIVE) {
            new PIDTunerTalon(mDriveMotor, Shuffleboard.getTab("mod " + moduleNumber + " Drive PID Tuner"));
        }

        if (moduleConstants.type == TestingType.STEER) {
            new PIDTunerTalon(mAngleMotor, Shuffleboard.getTab("mod " + moduleNumber + " Steer PID Tuner"));
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveSettings.SwerveDriveTrain.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveSettings.SwerveDriveTrain.wheelCircumference, SwerveSettings.SwerveDriveTrain.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveSettings.SwerveDriveTrain.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, SwerveSettings.SwerveDriveTrain.angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, SwerveSettings.SwerveDriveTrain.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveSettings.SwerveDriveTrain.angleMotorInvert);
        mAngleMotor.setNeutralMode(SwerveSettings.SwerveDriveTrain.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveSettings.SwerveDriveTrain.driveMotorInvert);
        mDriveMotor.setNeutralMode(SwerveSettings.SwerveDriveTrain.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = mDriveMotor.getObjectConvertedVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveSettings.SwerveDriveTrain.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getSwervePosition() {
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveSettings.SwerveDriveTrain.angleGearRatio));
        return new SwerveModulePosition(mDriveMotor.getObjectTotalDistanceTraveled(), angle);
    }

}