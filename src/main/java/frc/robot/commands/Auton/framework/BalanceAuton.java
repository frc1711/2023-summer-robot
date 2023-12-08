// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.framework;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Kinematics;

public class BalanceAuton extends CommandBase {
  
  Swerve swerveSubsystem;
  AutonDrive autonDriveCommand;
  Timer timer;
  Pneumatics pneumaticsSubsystem;

  public BalanceAuton(Swerve swerveSubsystem, Pneumatics pneumaticsSubsystem, Kinematics kinematics) {
    this.swerveSubsystem = swerveSubsystem;
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    autonDriveCommand = new AutonDrive(swerveSubsystem, kinematics, new Translation2d(), 0, 0); //TODO: determine these values
    timer = new Timer();
    addRequirements(swerveSubsystem, pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.stop();
    timer.restart();
    pneumaticsSubsystem.enableSubsystem();
  }

  boolean hasClimbedStation;
  double timeToPass;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasClimbedStation)
    swerveSubsystem.updateModules(new ChassisSpeeds(-.25, 0, 0), 1);

    if (swerveSubsystem.getGyroPitch() < -10 && !hasClimbedStation) {
      hasClimbedStation = true;
      timer.restart();
    } 

    if (swerveSubsystem.getGyroPitch() > -10 && hasClimbedStation) {
      timeToPass = timer.get();
      swerveSubsystem.updateModules(new ChassisSpeeds(.125, 0, 0), 1);
    }

    if (timer.hasElapsed(timeToPass)) swerveSubsystem.stop();
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
