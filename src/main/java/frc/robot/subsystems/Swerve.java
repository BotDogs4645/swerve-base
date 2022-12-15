package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Robot;
import frc.swervelib.util.SwerveSettings;
import frc.swervelib.util.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public ShuffleboardTab sub_tab;
    public double chassis_speed; // meters / second
    public Pose2d last_pose;
    public Instant last_instant;

    public Swerve() {
        this.gyro = new Pigeon2(SwerveSettings.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        this.chassis_speed = 0;
        this.sub_tab = Shuffleboard.getTab("swerve_tab2");

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveSettings.Swerve.Mod0.constants, sub_tab),
            new SwerveModule(1, SwerveSettings.Swerve.Mod1.constants, sub_tab),
            new SwerveModule(2, SwerveSettings.Swerve.Mod2.constants, sub_tab),
            new SwerveModule(3, SwerveSettings.Swerve.Mod3.constants, sub_tab)
        };

        this.swerveOdometry = new SwerveDriveOdometry(SwerveSettings.Swerve.swerveKinematics, getYaw(), getModulePositions());
        last_pose = swerveOdometry.getPoseMeters();
        last_instant = Instant.now();

        sub_tab.addDouble("IMU2", () -> getYaw().getDegrees())
        .withWidget(BuiltInWidgets.kGyro)
        .withSize(2, 2)
        .withPosition(1, 3);

        sub_tab.addDouble("Chassis Speedometer: MPS", () -> getChassisSpeed())
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0, "Max", SwerveSettings.Swerve.maxSpeed, "Show value", true))
        .withSize(4, 3);


        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule cur = mSwerveMods[i];
            ShuffleboardLayout layout = sub_tab.getLayout("mod " + cur.moduleNumber, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Label Position", "HIDDEN", "Number of columns", 1, "Number of rows", 2))
            .withPosition(SwerveSettings.ShuffleboardConstants.amp_rpm_placements.get(i)[0], SwerveSettings.ShuffleboardConstants.amp_rpm_placements.get(i)[1])
            .withSize(2, 2);

            layout.addDouble(1 + " " + i + 1, () -> Math.abs(cur.mDriveMotor.getObjectRotationsPerMinute()))
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", 800, "Show value", true));

            layout.addDouble(1 + " " + i, () -> cur.mDriveMotor.getSupplyCurrent())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("Min", 0, "Max", SwerveSettings.Swerve.drivePeakCurrentLimit));
        }
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
        return swerveOdometry.getPoseMeters();
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

    public double getChassisSpeed() {
        return chassis_speed;
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (SwerveSettings.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic() {
        Pose2d previous_pose = swerveOdometry.getPoseMeters();
        swerveOdometry.update(getYaw(), getModulePositions());
        Pose2d current_pose = swerveOdometry.getPoseMeters();

        Instant now = Instant.now();
        double elapsed_time = ((double)Duration.between(last_instant, now).getNano());
        chassis_speed = Math.abs(current_pose.getTranslation().getDistance(previous_pose.getTranslation())) / (elapsed_time / 1e9);

        last_instant = now;
    }
}