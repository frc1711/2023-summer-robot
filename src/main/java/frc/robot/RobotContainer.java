// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.SimpleAuton;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {
  
  private final Swerve swerveSubsystem;
  private final DriveCommand driveCommand;
  private final SwerveModule flModule, frModule, rlModule, rrModule;
  private final Translation2d flModuleTranslation, frModuleTranslation, rlModuleTranslation, rrModuleTranslation;
  private XboxController driverController;

  public RobotContainer() {

    driverController = new XboxController(0);

    flModuleTranslation = new Translation2d(-.43, .41);
    frModuleTranslation = new Translation2d(.43, .41);
    rlModuleTranslation = new Translation2d(-.43, -.41);
    rrModuleTranslation = new Translation2d(.43, -.41);

    flModule = new SwerveModule(
      IDMaps.flSteerMotorID, 
      IDMaps.flDriveMotorID, 
      IDMaps.flEncoderID, flModuleTranslation);
    frModule = new SwerveModule(
      IDMaps.frSteerMotorID, 
      IDMaps.frDriveMotorID, 
      IDMaps.frEncoderID, frModuleTranslation);
    rlModule = new SwerveModule(
      IDMaps.rlSteerMotorID, 
      IDMaps.rlDriveMotorID, 
      IDMaps.rlEncoderID, rlModuleTranslation);
    rrModule = new SwerveModule(
      IDMaps.rrSteerMotorID, 
      IDMaps.rrDriveMotorID, 
      IDMaps.rrEncoderID, rrModuleTranslation);

    swerveSubsystem = new Swerve(flModule, frModule, rlModule, rrModule);

    driveCommand = new DriveCommand(
      swerveSubsystem, 
      () -> driverController.getLeftX(), 
      () -> driverController.getLeftY(), 
      () -> driverController.getRightX(), 
      () -> driverController.getAButton(), 
      () -> driverController.getXButton());

    swerveSubsystem.setDefaultCommand(driveCommand);
  }

  public static void putSendable (String name, Sendable sendable) {
    Shuffleboard.getTab("Analysis Tab").add(name, sendable);
  }

  public static void putCommand (String name, CommandBase command, boolean canRunWhileDisabled) {
    putSendable(name, command.withName(name).ignoringDisable(canRunWhileDisabled));
  }

  public Command getAutonomousCommand() {
    return new SimpleAuton(swerveSubsystem);
  }
}
