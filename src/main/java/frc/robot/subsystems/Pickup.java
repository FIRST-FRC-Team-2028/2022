// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
  RelativeEncoder encoder;
  PneumaticHub pcm;
  //PneumaticsControlModule pcm;
  DoubleSolenoid arms;

  boolean rollersOn = false;
  static final int NUM_ENC = 200;
  double [] encoder_velocity = new double[NUM_ENC];
  int iter = 0;
  double enc_avg;

  public Pickup(PneumaticHub pcm) {
    rollers = new CANSparkMax(Constants.CANIDs.PICKUP_ROLLERS.getid(), MotorType.kBrushless);
    encoder = rollers.getEncoder();
    for(int i = 0; i < NUM_ENC; i++ ) encoder_velocity[i] = 0; 
    enc_avg = 0.;
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
    for(int i = 0; i < NUM_ENC; i++ ) encoder_velocity[i] = Constants.PICKUP_ROLLER_MOTOR_SPEED;
    enc_avg = Constants.PICKUP_ROLLER_MOTOR_SPEED;
    rollersOn = true;
  }
  public void runRollers(double speed){
    rollers.set(speed);
  }

  public void stopRollers() {
    rollers.set(0.);
    rollersOn = false;
  }

  /**
   * this method checks to see if pickup picked up
   * @return
   */
  public boolean hasCargo() {
    double IAMNOTDONE = 3;
    return (encoder.getVelocity() - enc_avg) < Constants.PICKUP_CARGO_INDICATION;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /**checks rpm of rollers when active */
    if(rollersOn){
      enc_avg = enc_avg  - encoder_velocity[iter]/NUM_ENC;
      encoder_velocity[iter] = encoder.getVelocity();
      iter = (iter+1)%NUM_ENC;
      enc_avg = enc_avg  + encoder.getVelocity()/NUM_ENC;
    }
  }
}
