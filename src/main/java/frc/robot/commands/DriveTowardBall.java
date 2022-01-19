// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Pixy2API.Pixy2;
import frc.robot.Pixy2API.Pixy2CCC;
import frc.robot.Pixy2API.Pixy2CCC.Block;
import frc.robot.subsystems.DriveSubsystem;

/** Assist the acquisition of balls by
 *  steering toward a ball with the aid
 *  of a camera.
 */
public class DriveTowardBall extends CommandBase {
  DriveSubsystem drive;
  Joystick joystick;
  Pixy2 camera;
  PIDController aimer;
  double  kp=.4;
  double kd=0.;
  double ki=0.;
  /** Creates a new DriveTowardBall. */
  public DriveTowardBall(DriveSubsystem drive, Pixy2 pixy, Joystick joystick) {
    this.drive = drive;
    this.joystick=joystick;
    this.camera=pixy;
    addRequirements(drive);
    aimer = new PIDController(kp, ki, kd);
    aimer.setSetpoint(0.);
    aimer.setIntegratorRange(-1., 1.);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /** execute()
   * look for the largest ball,
   * use the PIDController to drive the xValue to the center
   */
  public void execute() {
    Block biggest = null;
    int size = 0;
    double error;
    int numTargets = camera.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL,25);
    for (Block block : camera.getCCC().getBlockCache()) {
      if (block.getWidth() > size) {
        biggest=block;
        size=block.getWidth();
      }
    }
    if (biggest != null && biggest.getHeight() > 1) {
      error = (double)(biggest.getX() - CENTER_OF_CAMERA);
    }
    double stickY = aimer.calculate(error);
    drive.driveMe(joystick.getX(), stickY);
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
