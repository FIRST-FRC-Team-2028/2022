// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;

public class TurnoffPickup extends CommandBase {
  Pickup pickup;
  Magazine magazine;

  /** turn off the pickup rollers */
  public TurnoffPickup(Pickup subsystem /*,Magazine magazine*/) {
    this.pickup = subsystem;
    //this.magazine = magazine;
    addRequirements(/*magazine,*/ pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup.stopRollers();
    pickup.retract();
    //magazine.horizontaloff();
    magazine.horizontaloff();
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
