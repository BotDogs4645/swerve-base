// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.bdlib.pidtuner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TalonPIDBenchmarker extends CommandBase {
  /** Creates a new TalonPIDBenchmarker. */
  private PIDTunerTalon tuner;
  private TalonFX to_tune;
  private SimpleWidget bench_reporter;
  private GenericEntry time;

  private double time_to = -1;
  double CONVERSION_RATE = 600.0 / 2048.0;

  private double lower_bound;
  private double upper_bound;
  private int counter = 0;
  private double cur_velo = 0;
  private boolean truth = false;
  private boolean truth2 = false;
  private boolean reached_time = false;


  private double start_time;

  public TalonPIDBenchmarker(PIDTunerTalon tuner, TalonFX to_tune, SimpleWidget bench_reporter, GenericEntry time) {
    this.tuner = tuner;
    this.to_tune = to_tune;
    this.bench_reporter = bench_reporter;
    this.time = time;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_time = System.currentTimeMillis();
    this.lower_bound = Math.max(tuner.getRPMSetpoint() - ((tuner.getThreshold() / 100) * tuner.getRPMSetpoint()), 0);
    this.upper_bound = Math.min(tuner.getRPMSetpoint() + ((tuner.getThreshold() / 100) * tuner.getRPMSetpoint()), 6380);
  }

  public double ingetVelo() {
    return cur_velo;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    to_tune.set(ControlMode.Velocity, tuner.getRPMSetpoint() * CONVERSION_RATE);
    this.cur_velo = to_tune.getSelectedSensorVelocity() / CONVERSION_RATE;
    this.truth = cur_velo >= lower_bound;
    this.truth2 = cur_velo <= upper_bound;

    if(truth && truth2) {
      counter++;
    } else {
      counter = 0;
    }

    if(counter >= 100 && !reached_time) {
      time_to = System.currentTimeMillis() - start_time;
      reached_time = true; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.setValue((time_to - (counter * 20)) / 1000);
    bench_reporter.getEntry().setBoolean(false);
    tuner.setBenchValue(false);
    to_tune.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reached_time || !bench_reporter.getEntry().getBoolean(true);
  }
}
