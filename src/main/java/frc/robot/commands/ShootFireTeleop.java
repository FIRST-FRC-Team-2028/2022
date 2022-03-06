// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;


public class ShootFireTeleop extends CommandBase {
  Magazine magazine;
  Pickup pickup;
  Shooter turret;
  /** pushing ball into running shooter */
  public ShootFireTeleop(Magazine magazine, Pickup pickup, Shooter turret) {
    addRequirements(magazine, pickup, turret);
    this.magazine = magazine;
    this.pickup = pickup;
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  /** Starts magazine */
  @Override
  public void initialize() {
    magazine.verticalon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /** checks if shooter has shot then tells pickup*/
  public void execute() {
    if(turret.hasShot()) {
      pickup.usedAmmo();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
