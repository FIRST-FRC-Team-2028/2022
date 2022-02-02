// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
  double  kp=.15;
  double kd=0.;
  double ki=0.;
  int numTargets=0;
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
  public void initialize() {

    numTargets = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /** execute()
   * Look for the largest ball;
   * use the PIDController to drive the xValue to the center;
   * pixy.getX() returns center of block x relative to left edge of camera image;
   * PIDcontroller returns value in response to error;
   */
  public void execute() {
    Block biggest = null;
    int size = Constants.PIXY_MINIMUM_SIZE;
    double error=0.;
    numTargets = camera.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL,25);
    System.out.println("driveCamera sees "+numTargets);
    if (numTargets >1){
    String arString="";
    for (Block block : camera.getCCC().getBlockCache()) {
      if (block.getWidth() > size) {
        arString+=String.format("%4.2f ", (double)block.getHeight()/(double)block.getWidth());
        if(block.getHeight() < Constants.PIXY_TARGET_AR*block.getWidth() &&
           block.getHeight() > (int)((double)block.getWidth()/Constants.PIXY_TARGET_AR)) {
            biggest=block;
            size=block.getWidth();
        }
      }
    }
    SmartDashboard.putString("PixyResults", arString);
    if (biggest != null) {
      error = (double)(Constants.CENTER_OF_CAMERA - biggest.getX());
    }
    double stickX = aimer.calculate(error) / (double)Constants.CENTER_OF_CAMERA*5.;  // 
    SmartDashboard.putNumber("steerError", error);
    SmartDashboard.putNumber("steerPower", stickX);
    // limit turn power by forward power
    double forwardPower = joystick.getY();
    double sFP = Math.abs(forwardPower);
    if (sFP > Constants.DRIVE_TO_BALL_BUTTON){
      stickX = Math.max(stickX, -1.*sFP);
      stickX = Math.min(stickX, sFP);
    }else {
      stickX = Math.max(stickX, -1.*Constants.DRIVE_AIMER_SPEED_LIMIT);
      stickX = Math.min(stickX, Constants.DRIVE_AIMER_SPEED_LIMIT);
    }
    drive.driveMe(stickX, forwardPower);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (numTargets<1);
    return false;
  }
}
