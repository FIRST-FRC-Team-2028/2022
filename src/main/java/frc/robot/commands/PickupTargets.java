// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pickup;

public class PickupTargets extends CommandBase {
  /** Creates a new GetTargets. 
   * use the Pickup subsystem to get a ball
   * and the magazine subsystem draw it into the magazine
  */

  Pickup pickup;
  public PickupTargets(Pickup pickup) {
    this.pickup = pickup;
    addRequirements(pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup.deploy();
    pickup.runRollers();
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
