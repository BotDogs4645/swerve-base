// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervecommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings.SwerveDriveTrain;

public class RotateAroundAbsolutePoint extends CommandBase {
  Supplier<Translation2d> supplier;
  Swerve swerve;
  JoystickAxisAIO x, y, r;

  /** Creates a new RotationAroundRelativePoint. */
  public RotateAroundAbsolutePoint(Swerve swerve, JoystickAxisAIO x, JoystickAxisAIO y, JoystickAxisAIO r, Supplier<Translation2d> supplier) {
    // assume that coords are in absolute, meaning "field oriented"
    this.supplier = supplier;
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.r = r;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // map field oriented or "abs" coords to robot relative coords
    // requires SAME origin. dependent on blue team or red team
    Translation2d swerve_absolute_translation = swerve.getPose().getTranslation();
    Translation2d map = swerve_absolute_translation.minus(supplier.get());

    swerve.pointOrientedDrive(
      new Translation2d(y.getValue(), x.getValue()).times(SwerveDriveTrain.maxSpeed),
      r.getValue() * SwerveDriveTrain.maxAngularVelocity,
      map
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
