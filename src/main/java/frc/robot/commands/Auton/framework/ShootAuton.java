// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class ShootAuton extends CommandBase {

  Spinner spinnerSubsystem;
  int timeInSeconds;
  Timer timer;  
  
  public ShootAuton(Spinner spinnerSubsystem, int timeInSeconds) {
    this. spinnerSubsystem = spinnerSubsystem;
    this.timeInSeconds = timeInSeconds;
    timer = new Timer();
    addRequirements(spinnerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    spinnerSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!timer.hasElapsed(timeInSeconds)) {
      spinnerSubsystem.runSpinner(null);
    }

    else {
      spinnerSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    spinnerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
