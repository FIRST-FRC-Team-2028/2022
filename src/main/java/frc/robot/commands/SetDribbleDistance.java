// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class SetDribbleDistance extends CommandBase {
  Turret turret;
  /** Set Turret distance for just lightly shooting */
  public SetDribbleDistance(Turret turret) {
    addRequirements( turret);
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setdistance(Constants.SHOOTER_DRIBBLE_DISTANCE);
    turret.setelevation();
    turret.shooterdistance();
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
    return false;
  }
}
