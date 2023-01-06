// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.bd_util.driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PingRumble extends CommandBase {
  /** Creates a new PingRumble. */
  JoyRumbler rumbler;
  XboxController xbox;

  boolean waiting = false;
  int i = 0;

  public PingRumble(JoyRumbler rumbler, XboxController xbox) {
    this.rumbler = rumbler;
    this.xbox = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (rumbler.getCurrentCommand() != null) {
      waiting = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rumbler.getCurrentCommand() == null && waiting == true) {
      i++;
      if (i >= 100) {
        waiting = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        xbox.setRumble(RumbleType.kLeftRumble, .5);
        xbox.setRumble(RumbleType.kRightRumble, .5);
      }),
      new WaitCommand(1),
      new InstantCommand(() -> {
        xbox.setRumble(RumbleType.kLeftRumble, 0.0);
        xbox.setRumble(RumbleType.kRightRumble, 0.0);
      })
    ).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !waiting;
  }
}
