// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Auton.framework.ShootAuton;
import frc.robot.commands.Auton.framework.TaxiAuton;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Swerve;

public class PlaceAndTaxi extends CommandBase {
  
  Swerve swerveSubsystem;
  Spinner spinnerSubsystem;

  ShootAuton shoot;
  TaxiAuton taxi;

  public PlaceAndTaxi(Swerve swerveSubsystem, Spinner spinnerSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.spinnerSubsystem = spinnerSubsystem;
    shoot = new ShootAuton(spinnerSubsystem, 1);
    taxi = new TaxiAuton(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.stop();
    spinnerSubsystem.stop();
    shoot.andThen(taxi);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
