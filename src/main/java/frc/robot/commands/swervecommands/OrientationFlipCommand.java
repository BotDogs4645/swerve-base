// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OrientationFlipCommand extends ProfiledPIDCommand {
  Swerve swerve;

  /** Creates a new OrientationFlipCommand. */
  public OrientationFlipCommand(Swerve swerve, JoystickAxisAIO x, JoystickAxisAIO y) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.5,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(SwerveDriveTrain.maxAngularVelocity, 0)), // degrees per second lol
        // This should return the measurement
        () -> swerve.getYaw().getDegrees(),
        // This should return the goal (can also be a constant)
        swerve.getYaw().plus(Rotation2d.fromDegrees(180)).getDegrees(),
        // This uses the output
        (output, setpoint) -> {
          swerve.drive(
            new Translation2d(y.getValue(), x.getValue()).times(SwerveDriveTrain.maxSpeed), 
            Rotation2d.fromDegrees(output).getRadians(), 
            true,
            false
          );
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.swerve = swerve;
    addRequirements(swerve);
    getController()
        .setTolerance(.01, 5);
    getController()
        .enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    getController().reset(super.m_measurement.getAsDouble());
    getController().setGoal(
      swerve.getYaw().plus(Rotation2d.fromDegrees(180)).getDegrees()
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
