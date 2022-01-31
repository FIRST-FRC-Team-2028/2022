// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import javax.print.CancelablePrintJob;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** DriveSubsystem
 *  two motors on each side
 *  two-speed gear-box
 */
public class DriveSubsystem extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax leftFollower;
  CANSparkMax rightMotor;
  CANSparkMax rightFollower;
  Joystick joystick;
  //PneumaticHub pcm;
  PneumaticsControlModule pcm;
  DoubleSolenoid shifter;
  DifferentialDrive driverControl;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    leftMotor = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_LEADER.getid(), MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_FOLLOWER.getid(), MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.CANIDs.DRIVE_RIGHT_LEADER.getid(), MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.CANIDs.DRIVE_RIGHT_FOLLOWER.getid(), MotorType.kBrushless);
   
    leftMotor.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftMotor.setInverted(Constants.CANIDs.DRIVE_LEFT_LEADER.isInverted());
    leftFollower.setInverted(Constants.CANIDs.DRIVE_LEFT_FOLLOWER.isInverted());

    rightMotor.setInverted(Constants.CANIDs.DRIVE_RIGHT_LEADER.isInverted());
    rightFollower.setInverted(Constants.CANIDs.DRIVE_RIGHT_FOLLOWER.isInverted());

    leftFollower.follow(leftMotor);
    rightFollower.follow(rightMotor);

    driverControl = new DifferentialDrive(leftMotor, rightMotor);

    //pcm = new PneumaticHub();
    pcm = new PneumaticsControlModule(Constants.PNEUMATICS_CONTROL_MODULE);
    pcm.enableCompressorDigital();

    shifter = pcm.makeDoubleSolenoid(Constants.PneumaticChannel.DRIVE_LOW_GEAR.getChannel(), Constants.PneumaticChannel.DRIVE_HIGH_GEAR.getChannel());
    shifter.set(DoubleSolenoid.Value.kForward);
  }

  public void drive(double left, double right) {
    leftMotor.set(right);
    //leftFollower.setVoltage(left);
    rightMotor.set(right);
    //rightFollower.setVoltage(right);
  }

  /** DriveMe(forward+, right+) */
  public void driveMe (double stickX, double stickY) {
    if (stickX*stickX+stickY*stickY > Constants.SHIFTER_THRESHOLD 
       && shifter.get() == DoubleSolenoid.Value.kForward) {
      shifter.set(DoubleSolenoid.Value.kReverse);
    }
    else if (stickX*stickX+stickY*stickY < Constants.SHIFTER_THRESHOLD*.95 
       && shifter.get() == DoubleSolenoid.Value.kReverse) {
      shifter.set(DoubleSolenoid.Value.kForward);
    }
    driverControl.arcadeDrive(-stickX, stickY);
  }

  public void switchGears() {
    shifter.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
