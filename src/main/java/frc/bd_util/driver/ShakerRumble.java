// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.bd_util.driver;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShakerRumble extends CommandBase {
  /** Creates a new ShakerRumble. */
  JoyRumbler rumbler;
  BooleanSupplier reason;
  XboxController xbox;

  public ShakerRumble(JoyRumbler rumbler, XboxController xbox, BooleanSupplier reason) {
    this.rumbler = rumbler;
    this.reason = reason;
    this.xbox = xbox;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rumbler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      xbox.setRumble(RumbleType.kLeftRumble, 1.0);
      xbox.setRumble(RumbleType.kRightRumble, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xbox.setRumble(RumbleType.kLeftRumble, 0.0);
    xbox.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !reason.getAsBoolean();
  }
}
