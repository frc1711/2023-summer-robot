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

  double unoptimizedRotation, optimizedRotation, encoderOffset, encoderValue, steerSpeed, driveSpeed, drivePercent;

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
    steerPID.enableContinuousInput(-180, 180);
    this.motorMeters = motorMeters;
  }

  /** Uses the average RPM of the motor, along with the circumference of the wheel, 
   * to calculate an approximate voltage value when given a speed in meters per second 
   */
  private double maxSpeed = ((5500 / 60.) * .1) / 2;
  private double metersPerSecondToPercentage (double metersPerSecond) {
    return metersPerSecond / maxSpeed;
  }

  /**Sets the encoderOffset to the current value of the CANcoder. This value is 
   * later used to set a new zero position for the encoder. */
  public void resetEncoder () {
    encoderOffset = encoder.getAbsolutePosition();
  } 

  double finalAngle, unregulatedAngle;
  /**Uses the encoder offset, which is set using the resetEncoders() method, 
   * to determine the current position of the CANcoder */
  public Rotation2d getEncoderRotation () {
    finalAngle = -180;
    unregulatedAngle = encoder.getAbsolutePosition() - encoderOffset;
    if (unregulatedAngle < 0) {
      finalAngle += unregulatedAngle;
      return Rotation2d.fromDegrees(finalAngle);
    }
    else return Rotation2d.fromDegrees((encoder.getAbsolutePosition() - encoderOffset) - 180);
  }

  /**Takes in a SwerveModuleState, then uses a PID controller to calculate 
   * approximate values for the steerSpeed and the metersPerSecondToVoltage() 
   * method to calculate the driveVoltage. WIP*/
  public void update (SwerveModuleState desiredState) {
    unoptimizedRotation = desiredState.angle.getDegrees();
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getEncoderRotation());
    optimizedRotation = optimizedState.angle.getDegrees();
    double steerSpeed = steerPID.calculate(getEncoderRotation().getDegrees(), optimizedState.angle.getDegrees()); 
    this.steerSpeed = steerSpeed;
    steerMotor.set(steerSpeed);
    driveSpeed = optimizedState.speedMetersPerSecond;
    driveMotor.set(metersPerSecondToPercentage(optimizedState.speedMetersPerSecond));
    drivePercent = metersPerSecondToPercentage(optimizedState.speedMetersPerSecond);
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
    builder.addDoubleProperty("steer-Speed", () -> steerSpeed, null);
    builder.addDoubleProperty("drive-Speed", () -> driveSpeed, null);
    builder.addDoubleProperty("drive-Voltage", () -> drivePercent, null);
  }
}
