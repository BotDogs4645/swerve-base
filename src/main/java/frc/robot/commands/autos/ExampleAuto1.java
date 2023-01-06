package frc.robot.commands.autos;

import frc.robot.subsystems.Swerve;
import frc.swervelib.util.SwerveSettings.PathList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ExampleAuto1 extends SequentialCommandGroup {
    public ExampleAuto1(Swerve s_Swerve){
        Command paths = s_Swerve.getFullAutoPath(PathList.Path1, PathList.Path2);
        // or
        Command intro_path = s_Swerve.getSoloPathCommand(PathList.Path1, true);
        Command last_path = s_Swerve.getSoloPathCommand(PathList.Path2);

        addCommands(
            paths,
            intro_path,
            last_path
        );
    }
}
        addCommands(
            paths,
            intro_path,
            last_path
        );
    }
}