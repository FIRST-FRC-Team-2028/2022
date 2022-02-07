// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Mechanisms for picking balls off the floor */
public class Pickup extends SubsystemBase {
  /** Creates a new Pickup
   * consisting of solenoid to deploy the rollers,
   * motors to direct the ball.
  */
  CANSparkMax rollers;
  PneumaticHub pcm;
  //PneumaticsControlModule pcm;
  DoubleSolenoid arms;

  public Pickup(PneumaticHub pcm) {
    rollers = new CANSparkMax(Constants.CANIDs.PICKUP_ROLLERS.getid(), MotorType.kBrushless);
    this.pcm = pcm;
    arms = pcm.makeDoubleSolenoid(Constants.PneumaticChannel.PICKUP_EXTEND.getChannel(), Constants.PneumaticChannel.PICKUP_RETRACT.getChannel());
  }

  public void deploy () {
    if (arms.get() != DoubleSolenoid.Value.kReverse) {
      arms.set(DoubleSolenoid.Value.kReverse);
    }
  }
  public void retract () {
    if (arms.get() != DoubleSolenoid.Value.kForward) {
      arms.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void runRollers(){
    rollers.set(Constants.PICKUP_ROLLER_MOTOR_SPEED);
  }

  public void stopRollers() {
    rollers.set(0.);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
