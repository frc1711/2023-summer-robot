// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Swerve extends SubsystemBase {
  
  private SwerveModule 
    flModule,
    frModule,
    rlModule,
    rrModule;

  private AHRS gyro;

  private SwerveDriveKinematics kinematics;

  public Swerve(
    SwerveModule flModule,
    SwerveModule frModule,
    SwerveModule rlModule,
    SwerveModule rrModule,
    AHRS gyro
  ) {
    this.flModule = flModule;
    this.frModule = frModule;
    this.rlModule = rlModule;
    this.rrModule = rrModule;
    this.gyro = gyro;
    kinematics = new SwerveDriveKinematics(
        flModule.motorMeters,
        frModule.motorMeters,
        rlModule.motorMeters,
        rrModule.motorMeters
    );

    /**Create a new sendable field for each module*/
    RobotContainer.putSendable("fl-Module", flModule);
    RobotContainer.putSendable("fr-Module", frModule);
    RobotContainer.putSendable("rl-Module", rlModule);
    RobotContainer.putSendable("rr-Module", rrModule); 

    /**Create a new sendable command to reset the encoders */
    RobotContainer.putCommand("Reset Encoders", new InstantCommand(this::resetEncoders, this), true);
  }

  /**Runs the stop() method on each module */
  public void stop () {
    flModule.stop();
    frModule.stop();
    rlModule.stop();
    rrModule.stop();
  }

  /**Runs the resetEncoder() method on each module */
  public void resetEncoders() {
    flModule.resetEncoder();
    frModule.resetEncoder();
    rlModule.resetEncoder();
    rrModule.resetEncoder();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void updateModulesFieldRelative (ChassisSpeeds desiredVelocity) {
    ChassisSpeeds.fromFieldRelativeSpeeds(desiredVelocity, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredVelocity);
    flModule.update(moduleStates[0]);
    frModule.update(moduleStates[1]);
    rlModule.update(moduleStates[2]);
    rrModule.update(moduleStates[3]);
  }

  /**Updates each module using the reverse kinematics feature from SwerveDriveKinematics */
  public void updateModules (ChassisSpeeds desiredVelocity) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredVelocity);
    flModule.update(moduleStates[0]);
    frModule.update(moduleStates[1]);
    rlModule.update(moduleStates[2]);
    rrModule.update(moduleStates[3]);
  }

  @Override
  public void periodic() {
    
  }
  
}
