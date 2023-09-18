// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class BalanceAuton extends CommandBase {
  
  Swerve swerveSubsystem;
  AutonDrive autonDriveCommand;

  public BalanceAuton(Swerve swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    autonDriveCommand = new AutonDrive(swerveSubsystem, 3, .25, 0, 0); //TODO: determine these values
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autonDriveCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
