// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  CANCoder encoder;

  CANSparkMax driveMotor, steerMotor;

  Translation2d motorMeters;

  PIDController steerPID = new PIDController(.01, 0, 0);

  double unoptimizedRotation, optimizedRotation, encoderOffset, encoderValue;

  private CANSparkMax initializeMotor(int motorID) {
    CANSparkMax motor = new CANSparkMax(motorID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    return motor;
  }

  // private CANCoder initializeEncoder (int encoderID) {
  //   return new CANCoder(encoderID);
  // }

  public SwerveModule(int steerMotorID, int driveMotorID, int encoderID, Translation2d motorMeters) {
    encoder = new CANCoder(encoderID);
    driveMotor = initializeMotor(driveMotorID);
    steerMotor = initializeMotor(steerMotorID);
    this.motorMeters = motorMeters;
  }

  // private double voltageToMPS (double voltageRatio) {
  //   double rotationsPerSecond = (5500 / 60.) * voltageRatio;
  //   double wheelCircumferenceMeters = .1;
  //   return rotationsPerSecond * wheelCircumferenceMeters;
  // }

  private double metersPerSecondToVoltage (double metersPerSecond) {
    double rotationsPerSecond = metersPerSecond / .1;
     return rotationsPerSecond / (5500 / 60.); 
  }

  public void resetEncoder () {
    encoderOffset = encoder.getAbsolutePosition();
  } 

  public Rotation2d getEncoderRotation () {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - encoderOffset);
  }

  public void update (SwerveModuleState desiredState) {
    unoptimizedRotation = desiredState.angle.getDegrees();
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getEncoderRotation());
    optimizedRotation = optimizedState.angle.getDegrees();
    double steerSpeed = steerPID.calculate(getEncoderRotation().getDegrees(), optimizedState.angle.getDegrees()); 
    steerMotor.set(steerSpeed);
    driveMotor.setVoltage(metersPerSecondToVoltage(optimizedState.speedMetersPerSecond));
  }
  
  public void stop () {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    builder.addDoubleProperty("unop-rotation", () -> unoptimizedRotation, null);
    builder.addDoubleProperty("op-rotation", () -> optimizedRotation, null);
    builder.addDoubleProperty("actual-rotation", () -> getEncoderRotation().getDegrees(), null);
  }
}
