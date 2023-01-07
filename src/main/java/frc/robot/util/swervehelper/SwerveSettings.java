package frc.robot.util.swervehelper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class SwerveSettings {
    public static final CTREConfigs CTRE = new CTREConfigs();

    public static final class SwerveDriveTrain {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);
        public static final double bottomOfChassisHeight = Units.inchesToMeters(2);
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
        public static final int angleContinuousCurrentLimit = 30;
        public static final int anglePeakCurrentLimit = 50;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 40;
        public static final int drivePeakCurrentLimit = 70;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 14.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 12.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 1.5; // mps = 3.96
        public static final double maxAngularVelocity = 5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;


        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final double angleOffset = 177.45;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "Zero", angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final double angleOffset = 145.107;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "One", angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 9;
            public static final double angleOffset = 285.47;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "Two", angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 10;
            public static final double angleOffset = 258.574;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, "Three", angleMotorID, canCoderID, angleOffset);
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

      public static enum PathList {
        Path1("Path1", new PathConstraints(2, 2), 0.0),
        Path2("Path2", new PathConstraints(2, 2), 0.0);

        private String path_name;
        private PathConstraints constraints;
        private double relativeMagneticHeading;

        private PathList(String path_name, PathConstraints constraints, double relativeMagneticHeading) {
            this.path_name = path_name;
            this.constraints = constraints;
            this.relativeMagneticHeading = relativeMagneticHeading;
        }
       
        @Override
        public String toString() {
            return path_name;
        }

        public PathConstraints getConstraints() {
            return constraints;
        }

        public double getMagneticHeading() {
            return relativeMagneticHeading;
        }
      }

      public static final class ShuffleboardConstants {
        public enum BOARD_PLACEMENT {
            RPM0("RPM0", 0, 0),
            RPM1("RPM1", 8, 0),
            RPM2("RPM2", 0, 3),
            RPM3("RPM3", 8, 3),
            TEMP0("TEMP0", 2, 0),
            TEMP1("TEMP1", 7, 0),
            TEMP2("TEMP2", 2, 3),
            TEMP3("TEMP3", 7, 3);
    
            private String name;
            private int x;
            private int y;
    
            private BOARD_PLACEMENT(String name, int x, int y) {
                this.name = name;
                this.x = x;
                this.y = y;
            }
           
            @Override
            public String toString() {
                return name;
            }
    
            public int getX() {
                return x;
            }

            public int getY() {
                return y;
            }
          }
      }
}
