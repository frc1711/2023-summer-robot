// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.framework.ShootAuton;
import frc.robot.commands.Auton.framework.TaxiAuton;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Auton.framework.ShootAuton;
// import frc.robot.commands.Auton.framework.TaxiAuton;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Spinner.Node;

public class PlaceAndTaxi extends SequentialCommandGroup {
  
  Swerve swerveSubsystem;
  Pneumatics pneumaticsSubsystem;
  Spinner spinnerSubsystem;

  Timer timer;

  public PlaceAndTaxi(Swerve swerveSubsystem, Pneumatics pneumaticsSubsystem, Spinner spinnerSubsystem) {
    super (
      new ShootAuton(spinnerSubsystem),
      new TaxiAuton(swerveSubsystem, pneumaticsSubsystem)
    );
  //   this.swerveSubsystem = swerveSubsystem;
  //   this.spinnerSubsystem = spinnerSubsystem;
  //   // // timer = new Timer();
  //   // shoot = new ShootAuton(spinnerSubsystem, 1);
  //   // taxi = new TaxiAuton(swerveSubsystem, pneumaticsSubsystem);
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   // swerveSubsystem.stop();
  //   // spinnerSubsystem.stop();
  //   // timer.restart();
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  // // if (!timer.hasElapsed(1)) {
  // //   spinnerSubsystem.runSpinner(Node.HIGH);
  // // }

  // // else {
  // //   spinnerSubsystem.stop();
  // // }

  // // if (!timer.hasElapsed(3)) swerveSubsystem.updateModules(new ChassisSpeeds(-.5, 0, 0));

  // // else swerveSubsystem.stop(); //TODO: Determine timing for taxi auton
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   // swerveSubsystem.stop();
  //   // spinnerSubsystem.stop();
  //   // timer.stop();
  // }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
