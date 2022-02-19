// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AimTurretCommand extends CommandBase {
  Turret turret;
  double turretError;
  private static final double tolerance = 2.;  // pixels, presumably
  
  /** Aims turret towards hub */
  public AimTurretCommand(Turret m_turret) {
    turret = m_turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretError = turret.aimMe();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopAimer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(turretError) <tolerance);
  }
}
