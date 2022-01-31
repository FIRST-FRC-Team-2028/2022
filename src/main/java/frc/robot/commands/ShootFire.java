// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;


public class ShootFire extends CommandBase {
 
  Magazine magazine;
  /** Creates a new ShootFire. */
  public ShootFire(Magazine magazine) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(magazine);
    
    this.magazine = magazine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    magazine.verticalon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**move ball from magazine to turret */
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
