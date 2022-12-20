// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bd_util.driver.JoyRumbler;
import frc.bd_util.driver.JoyRumbler.RUMBLE_TYPE;
import frc.robot.commands.*;
import frc.robot.commands.autos.ExampleAuto1;
import frc.robot.commands.autos.ExampleCommand;
import frc.robot.subsystems.*;
import frc.swervelib.util.SwerveSettings;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  
  private final XboxController driver = new XboxController(0);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    boolean fieldRelative = true;
    boolean openLoop = true;
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, fieldRelative, openLoop));
    
    Shuffleboard.getTab("auto").add(autoChooser);

    JoyRumbler rumbler = new JoyRumbler(driver);
    rumbler.addRumbleScenario(() -> RobotController.getBatteryVoltage() < 10, RUMBLE_TYPE.SHAKER);

    // Configure the button bindings
    configureButtonBindings();
    // Configure autonomous mode
    configureAutonomous();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
  }

  private void configureAutonomous() {
    // when swerve reaches the event labeled "fire_ball", ExampleCommand will run.
    // note: swerve's path will not resume until ExampleCommand finishes unless,
    // it is set as a command that can run in parallel :)
    swerve.addEvent("fire_ball", new ExampleCommand());
    autoChooser.addOption("full auto 1", new ExampleAuto1(swerve));
    autoChooser.addOption("path important", swerve.getFullAutoPath(SwerveSettings.PATH_LIST.Path3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getChooser() {
    // An ExampleCommand will run in autonomous
    return autoChooser;
  }
}
