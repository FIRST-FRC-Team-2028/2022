// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  boolean dontCount = true;
  static final int NUM_ENC = 200;
  double [] encoder_velocity = new double[NUM_ENC];
  int iter = 0;
  double enc_avg;

  boolean engagedcargo;
  int ammo;

  public Pickup(PneumaticHub pcm) {
    rollers = new CANSparkMax(Constants.CANIDs.PICKUP_ROLLERS.getid(), MotorType.kBrushless);
    encoder = rollers.getEncoder();
    for(int i = 0; i < NUM_ENC; i++ ) encoder_velocity[i] = 0; 
    enc_avg = 0.;
    this.pcm = pcm;
    arms = pcm.makeDoubleSolenoid(Constants.PneumaticChannel.PICKUP_EXTEND.getChannel(), Constants.PneumaticChannel.PICKUP_RETRACT.getChannel());
    ammo = 1;
  }

  public CANSparkMax getrollers() {
    return rollers;
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
    engagedcargo = false;
    rollersOn = true;
  }
  public void runRollers(double speed){
    rollers.set(speed);
  }

  public void stopRollers() {
    rollers.set(0.);
    rollersOn = false;
    for(int i = 0; i < NUM_ENC; i++ ) encoder_velocity[i] = 0.;
    enc_avg = 0.;
    engagedcargo = false;
    dontCount = true;
    SmartDashboard.putBoolean("pickupDontCount", dontCount);
  }

  // allows the world to see roller velocity
  public double getRollerVelocity() {
     return encoder.getVelocity();
  }

  /**
   * this method checks to see if pickup picked up
   * @return
   */
  public boolean hasCargo() {
    //notice when the RPM drops from the running average
    //System.out.println("avg , current: " + enc_avg + " " + encoder.getVelocity() );
    if (!dontCount) {
      if(!engagedcargo && enc_avg - encoder.getVelocity() > Constants.PICKUP_CARGO_INDICATION) {
        engagedcargo = true;
        System.out.println("I was engaged");
        return false;
      }
      // notice when the RPM reverts to the running average
      if(engagedcargo && Math.abs(encoder.getVelocity() - enc_avg) < 10.) {
        ammo = ammo + 1;
        SmartDashboard.putNumber(" Magazine Ammo", ammo);
        engagedcargo = false;
        return true;
      } 
    }
    // When rollers spin up from not spinning, the method incorrectly detects ammo
    double I_NEED_CHECKED = 4.;
    return false;
  } 

  public int usedAmmo() {
    ammo = ammo - 1;
    return ammo;
  }
  /**
   * checks if we have gotten cargo then 
   * @return number of cargo balls
   */
  public int numCargo() {
    return ammo;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /**checks rpm of rollers when active */  
    if(rollersOn){
      if (dontCount) {  // don't look for cargo until rollers up to speed
       if (encoder.getVelocity() >= encoder_velocity[0]) {
        dontCount = false; //starts counting encoder velocity values
        SmartDashboard.putBoolean("pickupDontCount", dontCount);
       }
       encoder_velocity[0] = encoder.getVelocity();  // using one array value for detecting steady state
      } else {
        enc_avg = enc_avg  - encoder_velocity[iter]/NUM_ENC;
        encoder_velocity[iter] = encoder.getVelocity();
        iter = (iter+1)%NUM_ENC;
        enc_avg = enc_avg  + encoder.getVelocity()/NUM_ENC;
        SmartDashboard.putNumber("Pickup Roller Speed", encoder.getVelocity());
        SmartDashboard.putNumber("avg Roller Speed", enc_avg);
      }
    }
    SmartDashboard.putBoolean("engagedCargo", engagedcargo);
  }

  /** initialize agv RPM detector for testing */
  public void initAvg() {
    enc_avg = 6500;
    for (int i=0; i<NUM_ENC; i++) { encoder_velocity[i]=6500.; }
    iter = (iter+1)%NUM_ENC;
  }
  public void avg_update() {
    enc_avg = enc_avg  - encoder_velocity[iter]/NUM_ENC;
    encoder_velocity[iter] = encoder.getVelocity();
    iter = (iter+1)%NUM_ENC;
    enc_avg = enc_avg  + encoder.getVelocity()/NUM_ENC;
  }
}
