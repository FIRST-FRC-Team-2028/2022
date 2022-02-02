// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;

public class PickupTargets extends CommandBase {
  /** Creates a new GetTargets. 
   * use the Pickup subsystem to get a ball
  */

  Pickup subsystem;
  Magazine magazine;
  public PickupTargets(Pickup pickup,Magazine magazine) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.magazine = magazine;
    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.deploy();
    subsystem.runRollers();
    magazine.horizontalon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
