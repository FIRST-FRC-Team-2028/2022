// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveToCargo extends CommandBase {
  private DriveSubsystem drive;
  private double initPosition;
  private double currentPosition;
  private static final double tolerance= 2.;  // inches

  /** drive far enough to reach cargo near the tarmac
   * presuming initial robot placement is 
   *       facing the cargo
   *       bumpers just inside tarmac
   *  drive straight
   */
  public AutoDriveToCargo(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPosition = drive.getPosition();
    drive.driveMe(0.,Constants.DRIVE_LEAVE_TARMAC_SPEED);
    //SmartDashboard.putNumber("autoInitPos", initPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveMe(0.,Constants.DRIVE_LEAVE_TARMAC_SPEED);
    currentPosition = drive.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("autoPosition", currentPosition-initPosition);
    double error = Math.abs(currentPosition-initPosition - Constants.FIELD_TARMAC_TO_CARGO);
    //SmartDashboard.putNumber("autoPoserr", error);
    return  error < tolerance;
  }
}
