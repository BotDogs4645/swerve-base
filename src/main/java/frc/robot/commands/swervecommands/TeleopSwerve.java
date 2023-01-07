package frc.robot.commands.swervecommands;

import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings.SwerveDriveTrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private double rotation;
    private Translation2d translation;
    private boolean openLoop;
    
    private Swerve s_Swerve;

    private JoystickAxisAIO x;
    private JoystickAxisAIO y;
    private JoystickAxisAIO r;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, JoystickAxisAIO x, JoystickAxisAIO y, JoystickAxisAIO r, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.openLoop = openLoop;

        this.x = x;
        this.y = y;
        this.r = r;
    }

    @Override
    public void execute() {
        translation = new Translation2d(y.getValue(), x.getValue()).times(SwerveDriveTrain.maxSpeed);
        rotation = r.getValue() * SwerveDriveTrain.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, openLoop);
    }
}
