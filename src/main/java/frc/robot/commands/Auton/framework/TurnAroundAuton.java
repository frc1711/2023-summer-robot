// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.framework;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Kinematics;

public class TurnAroundAuton extends SequentialCommandGroup {
  
  Swerve swerveSubsystem;

  AutonDrive autonDrive;

  Timer timer;

  public TurnAroundAuton(Swerve swerveSubsystem, Kinematics kinematics) {
    super (new AutonDrive(swerveSubsystem, kinematics, new Translation2d(), 0, 0)); //TODO: Determine these values
  }
}
