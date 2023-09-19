// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Spinner.Node;

public class IntakeCommand extends CommandBase {
  Pneumatics pneumaticsSubsystem;

  Spinner spinnerSubsystem;

  BooleanSupplier runIntake, runIntakeReleased, shootLow, shootMid, shootHigh, runSpinner;

  Timer timer;

  public IntakeCommand(Pneumatics pneumaticsSubsystem, Spinner spinnerSubsystem, BooleanSupplier runIntake, BooleanSupplier runIntakeReleased, BooleanSupplier shootLow, BooleanSupplier shootMid, BooleanSupplier shootHigh, BooleanSupplier runSpinner) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.spinnerSubsystem = spinnerSubsystem;
    this.runIntake = runIntake;
    this.runIntakeReleased = runIntakeReleased;
    this.shootLow = shootLow;
    this.shootMid = shootMid;
    this.shootHigh = shootHigh;
    this.runSpinner = runSpinner;

    timer = new Timer();
    addRequirements(pneumaticsSubsystem, spinnerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumaticsSubsystem.enableSubsystem();
    timer.restart();
    spinnerSubsystem.stop();
  }

  Node node;

  @Override
  public void execute() {
    /**If the BooleanSupplier returns a true value, run the toggleSolenoid()
     method in the pneumaticsSubsystem */
      if (!runSpinner.getAsBoolean() && shootLow.getAsBoolean() && !shootMid.getAsBoolean() && !shootHigh.getAsBoolean()) node = Node.LOW;
      else if (!runSpinner.getAsBoolean() && shootMid.getAsBoolean() && !shootLow.getAsBoolean() && !shootHigh.getAsBoolean()) node = Node.MID;
      else if (!runSpinner.getAsBoolean() && shootHigh.getAsBoolean() && !shootLow.getAsBoolean() && !shootMid.getAsBoolean()) node = Node.HIGH;
      else if (runSpinner.getAsBoolean() && !shootHigh.getAsBoolean() && !shootLow.getAsBoolean() && !shootMid.getAsBoolean()) node = Node.INTAKE;
      else node = Node.STOP;

     
      if (runIntake.getAsBoolean()) {
        pneumaticsSubsystem.changeState(Value.kForward);
      }
      else if (runIntakeReleased.getAsBoolean()) {
        pneumaticsSubsystem.changeState(Value.kReverse);
      }

      spinnerSubsystem.runSpinner(node);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumaticsSubsystem.disableSubsystem();
    spinnerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
