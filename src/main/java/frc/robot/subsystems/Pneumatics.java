// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  
  Solenoid rightSolenoid, leftSolenoid;

  PneumaticsControlModule PCM;

  public Pneumatics(PneumaticsControlModule PCM) {
    this.PCM = PCM;

    rightSolenoid = PCM.makeSolenoid(0);
    leftSolenoid = PCM.makeSolenoid(0);
  }

  public void toggleSolenoid () {
    rightSolenoid.toggle();
    leftSolenoid.toggle();
  }

  public void enableSubsystem() {
    PCM.enableCompressorDigital();
  }

  public void disableSubsystem () {
    if (rightSolenoid.get()) rightSolenoid.toggle();
    if (leftSolenoid.get()) leftSolenoid.toggle();
    PCM.disableCompressor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
