// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class TurnAroundAuton extends CommandBase {
  
  Swerve swerveSubsystem;

  AutonDrive autonDrive;

  Timer timer;

  public TurnAroundAuton(Swerve swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    autonDrive = new AutonDrive(swerveSubsystem, 2, 0, 0, .5);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    swerveSubsystem.stop();
    autonDrive.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
