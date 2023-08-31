// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveCommand extends CommandBase {
  
  Swerve swerveSubsystem;

  DoubleSupplier xSpeed, ySpeed, thetaSpeed;

  BooleanSupplier slowMode, toggleFieldRelative, resetGyro;

  public DriveCommand(
    Swerve swerveSubsystem,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier thetaSpeed,
    BooleanSupplier slowMode,
    BooleanSupplier toggleFieldRelative,
    BooleanSupplier resetGyro
  ) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.thetaSpeed = thetaSpeed;
    this.slowMode = slowMode;
    this.toggleFieldRelative = toggleFieldRelative;
    this.resetGyro = resetGyro;

    addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {
    swerveSubsystem.stop();
  }

  double speedMultiplier;
  ChassisSpeeds chassisSpeeds;
  boolean isFieldRelative;
  @Override
  public void execute() {
    if (slowMode.getAsBoolean()) speedMultiplier = .5;
    else if (speedMultiplier != 1) speedMultiplier = 1;
    if (resetGyro.getAsBoolean()) swerveSubsystem.resetGyro();
    if (Math.abs(xSpeed.getAsDouble()) > .1 || Math.abs(ySpeed.getAsDouble()) > .1 || Math.abs(thetaSpeed.getAsDouble()) > .1) {
      chassisSpeeds = new ChassisSpeeds(
        xSpeed.getAsDouble() * speedMultiplier, 
        - ySpeed.getAsDouble() * speedMultiplier,
        thetaSpeed.getAsDouble() * speedMultiplier);
      if (toggleFieldRelative.getAsBoolean() && isFieldRelative) isFieldRelative = false;
      else if (toggleFieldRelative.getAsBoolean() && !isFieldRelative) isFieldRelative = true;
      if (isFieldRelative) swerveSubsystem.updateModulesFieldRelative(chassisSpeeds);
      else if (!isFieldRelative) swerveSubsystem.updateModules(chassisSpeeds);
    }
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
