package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Robot;
import frc.swervelib.util.SwerveSettings;
import frc.swervelib.util.SwerveSettings.PATH_LIST;
import frc.swervelib.util.SwerveSettings.ShuffleboardConstants.BOARD_PLACEMENT;
import frc.swervelib.util.SwerveModule;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public HashMap<String, Command> events = new HashMap<String, Command>();
    public HashMap<PATH_LIST, PathPlannerTrajectory> trajectories = new HashMap<PATH_LIST, PathPlannerTrajectory>();
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public ShuffleboardTab sub_tab;
    public SwerveAutoBuilder builder;
    public double chassis_speed; // meters / second
    public Pose2d last_pose;
    public Instant last_instant;

    /**
     * A swerve implementation using MK4 SDS modules, with full field oriented features.<p>
     * Original code from Team 264, heavily modified by Dave and Aidan
     * @return Swerve Drive subsystem
     */
    public Swerve() {
        // Declares and resets the Gyro to default. This wipes all settings about the gyro,
        // making it easily customizable in code only.
        this.gyro = new Pigeon2(SwerveSettings.Swerve.pigeonID);
        gyro.configFactoryDefault();

        // We also zero the gyro. In future implementations, we might remove this 
        // to allow us to save orientation data for matches.
        zeroGyro();

        // Used in the chassis speed calculation, check update() for more info
        this.chassis_speed = 0.0;

        // Gets us the swerve tab.
        this.sub_tab = Shuffleboard.getTab("swerve_tab");

        // These are our swerve modules. Each module has it's own constants
        // We also port the subsystem tab straight there so they can add their own information
        // We store them in an array so we can iterate through at any point.
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveSettings.Swerve.Mod0.constants, sub_tab),
            new SwerveModule(1, SwerveSettings.Swerve.Mod1.constants, sub_tab),
            new SwerveModule(2, SwerveSettings.Swerve.Mod2.constants, sub_tab),
            new SwerveModule(3, SwerveSettings.Swerve.Mod3.constants, sub_tab)
        };

        // SwerveDriveOdometry instances are used to calculate and keep track of the Robot's
        // pose, which is essentially the coordinates and orientation of the robot.
        this.swerveOdometry = new SwerveDriveOdometry(SwerveSettings.Swerve.swerveKinematics, getYaw(), getModulePositions());

        // Save our current pose and the instance for velocity calculations in update()
        last_pose = swerveOdometry.getPoseMeters();
        last_instant = Instant.now();

        // The SwerveAutoBuilder is used to create paths for this particular swerve drive.
        // All the PID is contained here, and no other commands relating to PathPlanner have to be created.
        // This builder also uses events, which is explained in the JavaDocs for the addEvents command.
        this.builder = new SwerveAutoBuilder(
            this::getPose, // Pose2d supplier
            this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            SwerveSettings.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(2.5, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            this::setModuleStates, // Module states consumer used to output to the drive subsystem
            events,
            this // The drive subsystem. Used to properly set the requirements of path following commands
        );

        // Gyro initialization, we use a Pigeon and we want the Yaw in degrees so we can directly
        // show it on Shuffleboard.
        // sub_tab.addInteger("Pigeon IMU", () -> (int)getYaw().getDegrees())
        // .withSize(2, 2)
        // .withPosition(4, 3)
        // .withWidget(BuiltInWidgets.kGyro);

        // Our speedometer, uses the chassis_speed variable.
        sub_tab.addDouble("Chassis Speedometer: MPS", () -> getChassisSpeed())
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0.0, "Max", SwerveSettings.Swerve.maxSpeed, "Show value", true))
        .withSize(4, 3)
        .withPosition(3, 0);

        // Add RPM and current calculations for each module and place them on the Shuffleboard
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule cur = mSwerveMods[i];
            BOARD_PLACEMENT placement = BOARD_PLACEMENT.valueOf("RPM" + i);
            ShuffleboardLayout layout = sub_tab.getLayout("mod " + cur.moduleNumber, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 2))
            .withPosition(placement.getX(), placement.getY())
            .withSize(2, 2);

            layout.addDouble("RPM " + i, () -> Math.abs(cur.mDriveMotor.getObjectRotationsPerMinute()))
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", 800, "Show value", true));

            layout.addDouble("AMPS " + i, () -> cur.mDriveMotor.getSupplyCurrent())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("Min", 0, "Max", SwerveSettings.Swerve.drivePeakCurrentLimit));
        }

        // iterates through the available path enums, and then puts them into the available path
        // hashmap.
        for (PATH_LIST enu: PATH_LIST.values()) {
            trajectories.put(enu, PathPlanner.loadPath(enu.toString(), enu.getConstraints()));
        }
    }

    /**
     * Used to port joystick input from the Drive command into the system.
     * @param translation The value of requested meters of translation from the current point. 
     * @param rotation The value of requested rotation in radians
     * @param fieldRelative Determines whether or not to establish field oriented control
     * @param isOpenLoop Determines whether or not to use PID for values; true = yes, false = no.
     * 
     */
    // TODO add orientation lock w/ joystickbutton
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // How module states work is this: we have a current position, a translation that we want to do, and a rotation vector
        // that we also want to do. From there, we take our current position add the translation and rotation and using
        // inverse kinematics, it returns each module's "state", or rather what direction to rotate to and what velocity to
        // spin at.
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

        setModuleStates(swerveModuleStates, isOpenLoop);
    }    


    /**
     * Sets each module state, requires all states. This version always has openLoop at true,
     * because it is used for autonomous, which requires PID.
     * @param desiredStates Array of module states 
     * 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSettings.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    /**
     * Sets each module state, requires all states. This version has openLoop as optional, which
     * means it can be used with a joystick.
     * @param desiredStates Array of module states
     * @param isOpenLoop Determines if it uses PID
     * 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSettings.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Gets the current {@link Pose2d}, with all translation values in meters.
     * @return the current pose in meters
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a specified {@link Pose2d}.
     * @param pose Resets the robot's pose to this pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Gets the position of all the modules
     * @return Returns every swerve module's position in {@link SwerveModulePosition} form.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getSwervePosition();
        }
        return states;
    }

    /**
     * Resets the gyro, used for FOC.
     */
    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    /**
     * Internal usage for compatability with the {@link DriverProfile} system.
     * @param settings The settings HashMap from a driver settings json file.
     */
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

    /**
     * Gets the chassis's speed
     * @return chassis speed in meters / second
     */
    public double getChassisSpeed() {
        return chassis_speed;
    }

    /**
     * Gets the yaw with special math to make the gyro non-continous
     * Reason why is because CTRE does not do continious, while WPILib does.
     * @return Rotation2d representing the yaw
     */
    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (SwerveSettings.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    /**
     * Adds an event to the event list. The event list directly corresponds to the strings you give
     * events in the PathPlanner application.
     * @param event_name The same name as the one you assigned in PathPlanner
     * @param command_to_execute The {@link Command} you want to execute at this event fire
     */
    public void addEvent(String event_name, Command command_to_execute) {
        events.put(event_name, command_to_execute);
    }

    /**
     * Used to get a path command from only one PATH_LIST trajectory. This variant is to get
     * the Command for the first path, is_first has to be true.
     * @param traj_path The {@link PATH_LIST} trajectory from the established list of paths to use
     * @param is_first The variable that determines whether or not to reset the gyro at the start.
     * @return the usable {@link Command}
     * 
     */
    public Command getSoloPathCommand(PATH_LIST traj_path, boolean is_first) {
        PathPlannerTrajectory traj = trajectories.get(traj_path);
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(is_first){
                  this.resetOdometry(traj.getInitialHolonomicPose());
              }
            }),
            builder.followPathWithEvents(traj)
        );
    }

    /**
     * Used to get a path command from only one PATH_LIST trajectory.
     * @param path The {@link PATH_LIST} trajectory from the established list of paths to use
     * @return the usable {@link Command} 
     */
    public Command getSoloPathCommand(PATH_LIST path) {
        return builder.followPathWithEvents(trajectories.get(path));
    }

    /**
     * Used to get a single command that runs all supplied trajectory {@link PATH_LIST} trajectories.<p>
     * DOES NOT reset odometry at the beginning.
     * @param traj A list of {@link PATH_LIST} trajectories to create a {@link Command} from
     * @return the usable {@link Command}
     */
    public Command getFullAutoPath(PATH_LIST... traj) {
        ArrayList<PathPlannerTrajectory> paths = new ArrayList<PathPlannerTrajectory>();
        for (int i = 0; i < traj.length; i++) {
            paths.add(trajectories.get(traj[i]));
        } 

        SequentialCommandGroup sequential = new SequentialCommandGroup(
            new InstantCommand(() -> this.resetOdometry(paths.get(0).getInitialHolonomicPose())),
            builder.fullAuto(paths)
        );

        return sequential;
    }

    @Override
    public void periodic() {
        // Grabs the previous post of the robot, before the update.
        Pose2d previous_pose = swerveOdometry.getPoseMeters();
        swerveOdometry.update(getYaw(), getModulePositions());

        // Grabs the new pose of the robot, after the update.
        Pose2d current_pose = swerveOdometry.getPoseMeters();

        // Grabs the current time of the system
        Instant now = Instant.now();
        // Determines how much time has passed.
        double elapsed_time = ((double)Duration.between(last_instant, now).getNano());
        // Does some math to essentially get the distance between the translation, and then divide it by
        // the elapsed time. Since getNano() nanoseconds, to get it to seconds we need to divide by 1e9.
        chassis_speed = Math.abs(current_pose.getTranslation().getDistance(previous_pose.getTranslation())) / (elapsed_time / 1e9);

        // sets the last instance to now for the next iteration of periodic.
        last_instant = now;
    }
}