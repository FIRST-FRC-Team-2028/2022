// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ShiftGears extends CommandBase {
  DriveSubsystem drive;
  Timer timeTilRepeat;
  /** Switches gears*/
  public ShiftGears(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    timeTilRepeat = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.switchGears();
    timeTilRepeat.start();
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
    return timeTilRepeat.hasElapsed(0.5);
  }
}
