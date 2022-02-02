// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends SubsystemBase {
  CANSparkMax climbMotor;
  //PneumaticHub pcm;
  PneumaticsControlModule pcm;
  DoubleSolenoid shifter;
  double starttime;
  /** Creates a new Climber. */
  public Climber() {
    climbMotor = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_LEADER.getid(), MotorType.kBrushless);
  }

   void resetmotors() {
   
    climbMotor.restoreFactoryDefaults();
    climbMotor.setInverted(Constants.CANIDs.DRIVE_LEFT_LEADER.isInverted());
  }
  /** deploys climber  */
  public void deployclimber() {

  }

//** pulls robot up to the bar */
public void pullup() {
starttime = Timer.getFPGATimestamp();
}

public boolean amidone() {
return (Timer.getFPGATimestamp()- starttime) > (Constants.CLIMBER_TIME_TO_CLIMB);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
