// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private final Joystick m_joystick;
  private int iterCount;

  /** drives robot */
  public DefaultDriveCommand(DriveSubsystem drive, Joystick joystick) {
    m_drive = drive;
    m_joystick = joystick;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    iterCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn;
    if (Constants.JOYSTICK_EXTREME3D) {
      turn = m_joystick.getZ();
    } else {
      turn = m_joystick.getX();
    }
     m_drive.driveMe(turn, -m_joystick.getY());
     if (m_joystick.getRawButton(Constants.SHIFTER_BUTTON)){
       m_drive.switchGears();
     }
     iterCount++;
     SmartDashboard.putNumber("defCom iter", (double)iterCount);
     SmartDashboard.putNumber("defComX", m_joystick.getX());
     System.out.println("here is getX "+iterCount+" "+m_joystick.getX());
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
