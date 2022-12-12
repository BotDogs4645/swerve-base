package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.swervelib.util.SwerveConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private XboxController controller;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(SwerveConstants.leftX);
        double xAxis = -controller.getRawAxis(SwerveConstants.leftY);
        double rAxis = -controller.getRawAxis(SwerveConstants.rightX);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < SwerveConstants.deadzone) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < SwerveConstants.deadzone) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < SwerveConstants.deadzone) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(SwerveConstants.Swerve.maxSpeed);
        rotation = rAxis * SwerveConstants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
