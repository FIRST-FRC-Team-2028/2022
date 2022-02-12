// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


public class AutoLeaveTarmacTimed extends CommandBase {
  DriveSubsystem drive;
  Timer timer;


  /** Autonomously drive forward ~7 feet
   *  so that all of its bumpers have completely left the tarmac.
   *  TODO:
   *     Test to determine drive speed
   *     Test to reliably get time to drive 7 feet 
   *     Define Constants
   */
  public AutoLeaveTarmacTimed(DriveSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(Constants.DRIVE_LEAVE_TARMAC_SPEED, Constants.DRIVE_LEAVE_TARMAC_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.hasElapsed(Constants.DRIVE_TIME_TO_LEAVE_TARMAC);
  }
}
