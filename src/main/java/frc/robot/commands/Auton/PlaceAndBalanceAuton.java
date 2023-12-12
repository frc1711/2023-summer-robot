// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.framework.BalanceAuton;
import frc.robot.commands.Auton.framework.ShootAuton;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Kinematics;

public class PlaceAndBalanceAuton extends SequentialCommandGroup {

  public PlaceAndBalanceAuton(Swerve swerveSubsystem, Spinner spinnerSubsystem, Pneumatics pneumaticsSubsystem, Kinematics kinematics, AHRS gyro) {
    
    super (
      new ShootAuton(spinnerSubsystem),
      new BalanceAuton(swerveSubsystem, pneumaticsSubsystem, kinematics, gyro)
    );
  }

}
