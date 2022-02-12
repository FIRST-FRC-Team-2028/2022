// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLeaveTarmacDistance extends CommandBase {
  private DriveSubsystem drive;
  private double initPosition;
  private double currentPosition;
  private static final double tolerance= .1;  //feet

  /** drive far enough to leave tarmac */
  public AutoLeaveTarmacDistance(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPosition = drive.getPosition();
    drive.driveMe(0.,Constants.DRIVE_LEAVE_TARMAC_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = drive.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(currentPosition-initPosition - Constants.ROBOT_LENGTH*0.5) < tolerance;
  }
}
