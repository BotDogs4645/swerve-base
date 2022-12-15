package frc.swervelib.util;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class SwerveSettings {
    public static double deadzone = 0.04;

    public static final int leftX = 1;
    public static final int leftY = 0;
    public static final int rightX = 2;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final boolean testing = true;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double feetInAMile = 5280;
        public static final double secondsInAnHour = 3600;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 55;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 14.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.025;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static double maxSpeed = 4.5; //meters per second * 4.5
        public static double maxAngularVelocity = 12;

        /* Neutral Modes */
        public static NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public enum TESTING_TYPE {
            NONE,
            STEER,
            DRIVE
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final double angleOffset = 354.64;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "Zero", angleMotorID, canCoderID, angleOffset, TESTING_TYPE.NONE);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final double angleOffset = 324.32;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "One", angleMotorID, canCoderID, angleOffset, TESTING_TYPE.NONE);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 9;
            public static final double angleOffset = 102.92;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "Two", angleMotorID, canCoderID, angleOffset, TESTING_TYPE.NONE);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 10;
            public static final double angleOffset = 79.80;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "Three", angleMotorID, canCoderID, angleOffset, TESTING_TYPE.NONE);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

      public static final class ShuffleboardConstants {
        public static final ArrayList<int[]> amp_rpm_placements = new ArrayList<>();
        public static final ArrayList<int[]> temp_placements = new ArrayList<>();
        static {
            amp_rpm_placements.add(new int[] {5,0});
            amp_rpm_placements.add(new int[] {7,0});
            amp_rpm_placements.add(new int[] {5,3});
            amp_rpm_placements.add(new int[] {7,3});

            temp_placements.add(new int[] {4, 0});
            temp_placements.add(new int[] {9, 0});
            temp_placements.add(new int[] {4, 3});
            temp_placements.add(new int[] {9, 3});
        }
      }
}
