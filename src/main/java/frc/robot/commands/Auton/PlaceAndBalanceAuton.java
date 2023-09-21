// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.framework.AutonDrive;
import frc.robot.commands.Auton.framework.ShootAuton;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Swerve;

public class PlaceAndBalanceAuton extends SequentialCommandGroup {
  
  Swerve swerveSubsystem;
  AutonDrive driveToStation;
  ShootAuton shoot;
  Spinner spinnerSubsystem;

  public PlaceAndBalanceAuton(Swerve swerveSubsystem, Spinner spinnerSubsystem) {
    
    super (
      new ShootAuton(spinnerSubsystem),
      new AutonDrive(swerveSubsystem, 4, -.25, 0, 0)
    );
  }

}
