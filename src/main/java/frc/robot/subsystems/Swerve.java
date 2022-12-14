package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Robot;
import frc.swervelib.util.SwerveSettings;
import frc.swervelib.util.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public ShuffleboardTab sub_tab;

    
    public Swerve() {
        gyro = new PigeonIMU(SwerveSettings.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        sub_tab = Shuffleboard.getTab("swerve_tab2");

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveSettings.Swerve.Mod0.constants, sub_tab),
            new SwerveModule(1, SwerveSettings.Swerve.Mod1.constants, sub_tab),
            new SwerveModule(2, SwerveSettings.Swerve.Mod2.constants, sub_tab),
            new SwerveModule(3, SwerveSettings.Swerve.Mod3.constants, sub_tab)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(SwerveSettings.Swerve.swerveKinematics, getYaw(), getModulePositions(), getPose());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveSettings.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveSettings.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSettings.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getSwervePosition();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    public void updatePreferences(HashMap<String, String> settings) {
        SwerveSettings.deadzone = Double.parseDouble(settings.get("stick_deadband"));
        Robot.ctreConfigs.swerveDriveFXConfig.openloopRamp = Double.parseDouble(settings.get("open_ramp"));
        Robot.ctreConfigs.swerveDriveFXConfig.closedloopRamp = Double.parseDouble(settings.get("closed_ramp"));
        SwerveSettings.Swerve.maxSpeed = Double.parseDouble(settings.get("max_speed"));
        SwerveSettings.Swerve.maxAngularVelocity = Double.parseDouble(settings.get("max_angular_velocity"));
        SwerveSettings.Swerve.angleNeutralMode = Boolean.parseBoolean(settings.get("brake_angle")) ? NeutralMode.Brake : NeutralMode.Coast;
        SwerveSettings.Swerve.driveNeutralMode = Boolean.parseBoolean(settings.get("brake_drive")) ? NeutralMode.Brake : NeutralMode.Coast;

        for (SwerveModule swerve_module: mSwerveMods) {
            swerve_module.reapplyConfig();
        }
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (SwerveSettings.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
    }
}