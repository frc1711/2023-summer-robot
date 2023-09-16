// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  
  CANSparkMax leftSpinnerMotor, rightSpinnerMotor;

  public Spinner(int leftMotorID, int rightMotorID) {
    leftSpinnerMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    rightSpinnerMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    leftSpinnerMotor.setIdleMode(IdleMode.kBrake);
    rightSpinnerMotor.setIdleMode(IdleMode.kBrake);
  }

  public enum Node {
    LOW,
    MID,
    HIGH;
  }

  double reverseMultiplier;
  public void runSpinner (Node node) {
    if (node == Node.HIGH) reverseMultiplier = -2;
    if (node == Node.MID) reverseMultiplier = -1.5;
    if (node == Node.LOW) reverseMultiplier = -1;
    else reverseMultiplier = 1;
    leftSpinnerMotor.set(.5 * reverseMultiplier); //TODO: Run testing to determine optimal speed
    rightSpinnerMotor.set(.5 * reverseMultiplier);
  }

  public void stop () {
    leftSpinnerMotor.stopMotor();
    rightSpinnerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
