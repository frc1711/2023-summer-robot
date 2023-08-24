// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  
  CANSparkMax spinnerMotor;

  public Spinner(CANSparkMax spinnerMotor) {
    this.spinnerMotor = spinnerMotor;
  }

  public void runSpinner () {
    spinnerMotor.set(50); //TODO: Run testing to determine optimal speed
  }

  public void stop () {
    spinnerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
