// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.framework;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Kinematics;

public class AutonDrive extends CommandBase {

  Swerve swerveSubsystem;
  Kinematics kinematics;
  // Timer timer;
  double timeInSeconds, xSpeedMPS, ySpeedMPS, thetaSpeedMPS, desiredRotation, currentRotation;
  Translation2d desiredPosition;
  PIDController xPID, yPID, thetaPID;

  public AutonDrive(Swerve swerveSubsystem, Kinematics kinematics, Translation2d desiredPosition, double desiredRotation, double currentRotation /**double timeInSeconds, double xSpeedMPS, double ySpeedMPS, double thetaSpeedMPS*/) {
    this.swerveSubsystem = swerveSubsystem;
    this.kinematics = kinematics;
    this.desiredPosition = desiredPosition;
    // this.timer = new Timer();
    // this.timeInSeconds = timeInSeconds;
    xPID = new PIDController(0.05, 0, 0);
    yPID = new PIDController(0.05, 0, 0);
    thetaPID = new PIDController(0.05, 0, 0);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    /**Restarts the timer which is used to run the auton cycle. */
    // timer.restart();
    swerveSubsystem.stop();
  }

  @Override
  public void execute() {

    
    
    swerveSubsystem.updateModules(new ChassisSpeeds(
                  xPID.calculate(kinematics.getRobotDisplacementX(), desiredPosition.getX()), 
                  yPID.calculate(kinematics.getRobotDisplacementY(), desiredPosition.getY()), 
                  thetaPID.calculate(currentRotation, desiredRotation)), 
                  1);
    /**Checks if the given time has passed since the timer has been 
     * reset. If not, run the updateModules() method to move the robot*/
    // if (!timer.hasElapsed(timeInSeconds)) swerveSubsystem.updateModules(new ChassisSpeeds(xSpeedMPS, ySpeedMPS, thetaSpeedMPS), 1);
    // else swerveSubsystem.stop();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
    // timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
