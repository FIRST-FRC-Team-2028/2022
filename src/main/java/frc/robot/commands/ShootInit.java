// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**  sets a motors speed amd elavation angle based off of distance from the target  */
public class ShootInit extends CommandBase {
Turret shooter;
  /** Creates a new Shoot. */
  public ShootInit(Turret shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  /** sets motor speed and elavation angle */
  public void initialize() {
    // gotta get distance from hub to robot
    //distance = robot.getDistance_from_hub();
    shooter.setelevation();
    shooter.shooterdistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**move a ball from magazine to turret */
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  /** turns off motor  */
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  /** when the shhoter is positioned and up to speed */
  public boolean isFinished() {
    return shooter.iselevatordistance() && shooter.isatSpeed();
  }
}
