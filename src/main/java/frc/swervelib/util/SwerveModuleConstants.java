package frc.swervelib.util;

import frc.swervelib.util.SwerveSettings.SwerveDriveTrain.TestingType;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public final String name;
    public final TestingType type;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param testing are we pid tuning?
     */
    public SwerveModuleConstants(int driveMotorID, String name, int angleMotorID, int canCoderID, double angleOffset, TestingType type) {
        this.driveMotorID = driveMotorID;
        this.name = name;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.type = type;
    }
}
