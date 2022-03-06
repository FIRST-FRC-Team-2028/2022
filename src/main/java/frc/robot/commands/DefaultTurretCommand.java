// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class DefaultTurretCommand extends CommandBase {
  Turret turret;
  Joystick buttons;
  /** Creates a new DefaultTurretCommand. */
  public DefaultTurretCommand(Turret turret, Joystick buttons) {
    addRequirements(turret);
    this.turret = turret;
    this.buttons = buttons;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (buttons.getRawButton(Constants.TURRETCW_BUTTON)){
         turret.turretCW(1.);
      }else if (buttons.getRawButton(Constants.TURRETCCW_BUTTON)){
         turret.turretCW(-1.);
      }else{
         turret.turretCW(0.);
      }
      if (buttons.getRawButton(Constants.TURRET_FINE_BUTTON)){
         turret.turretFine(true);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
