// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SimpleAuton extends CommandBase {

  Swerve swerveSubsystem;
  Timer timer;

  public SimpleAuton(Swerve swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.timer = new Timer();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    /**Restarts the timer which is used to run the auton cycle. */
    timer.restart();
    swerveSubsystem.stop();
  }

  @Override
  public void execute() {
    
    /**Checks if 3 seconds have passed since the timer has been 
     * reset. If not, run the updateModules() method to move the robot*/
    if (!timer.hasElapsed(3)) swerveSubsystem.updateModules(new ChassisSpeeds(1, 0, 0));
    else swerveSubsystem.stop();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
