// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Swerve;

public class PlaceAndBalanceAuton extends CommandBase {
  
  Swerve swerveSubsystem;
  AutonDrive driveToStation;
  ShootAuton shoot;
  Spinner spinnerSubsystem;

  public PlaceAndBalanceAuton(Swerve swerveSubsystem, Spinner spinnerSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.spinnerSubsystem = spinnerSubsystem;
    driveToStation = new AutonDrive(swerveSubsystem, 3, .5, 0, 0); //TODO: Determine these values
    shoot = new ShootAuton(spinnerSubsystem, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.stop();
    spinnerSubsystem.stop();
    
    shoot.andThen(driveToStation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
    spinnerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
