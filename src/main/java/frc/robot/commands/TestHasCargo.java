// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pickup;

public class TestHasCargo extends CommandBase {
  /** Creates a new TestHasCargo. */
  Pickup pickup;
  public TestHasCargo(Pickup pickup) {
    addRequirements(pickup);
    this.pickup=pickup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup.hasCargo();
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
    return pickup.hasCargo();
  }
}
