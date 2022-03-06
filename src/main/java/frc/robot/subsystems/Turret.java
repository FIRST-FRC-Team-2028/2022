// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.QuadraticFitter;
import frc.robot.Pixy2API.Pixy2;
import frc.robot.Pixy2API.Pixy2CCC;
import frc.robot.Pixy2API.Pixy2CCC.Block;
import frc.robot.Pixy2API.links.I2CLink;

/* 
 * public methods:
 *     turret:
 *        aimer
 *        turretCW
 *        turretFine
 */

/** Turret points toward the hub and shoot balls into it.
 */
public class Turret extends SubsystemBase {
  CANSparkMax turretMotor;
  RelativeEncoder turretencoder;
  double turretupperlimit;
  double turretlowerlimit;
  boolean turretiszeroed = true; // starts as true so autonomous doesnt move it
  double turretSpeed;
  double turret_position;
  double turret_position_two;
  boolean looking_for_position_two = false;
  boolean badposition = true;

  AnalogInput pixyCam;
  Pixy2 pixyCamI2C;
  PIDController aimer;
  double kp=.3;
  double ki=0.;
  double kd=0.;

  RobotContainer robotContainer;

  public Turret(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    if (Constants.TURRET_AVAILABLE){

      turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_AZIMUTH.getid(), MotorType.kBrushless);
      turretMotor.restoreFactoryDefaults();
      turretMotor.setInverted(Constants.CANIDs.TURRET_AZIMUTH.isInverted());
      turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      turretSpeed = Constants.TURRET_MOTOR_SPEED;
      //turretencoder =  turretMotor.getEncoder();
      //turretencoder.setPositionConversionFactor(1./Constants.TURRET_ENCODER_RATIO);
  
//       turretencoder.setPosition(0.);

// turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
// turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
// turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 45);
// turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -45);

      //turretswitch = new AnalogInput(Constants.TURRET_SWITCH_CHANNEL);
      //turretswitch = turretMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
      
    }

    //SmartDashboard.putNumber("Turret Alert", 0.);  // Tells driver that turret is located in a safe region

  }


  public void turretFine(boolean fine) {
    //System.out.println("fine");
    if (fine) turretSpeed = Constants.TURRET_MOTOR_SLOW_SPEED;
    else turretSpeed = Constants.TURRET_MOTOR_SPEED;
  }
  /** move turret
   * @param speed [+1, 0., -1] positive clockwise around up vector
   */
  public void turretCW(double speed){
    //System.out.println("CW speed"+speed+" "+turretSpeed);

    turretMotor.set(speed*turretSpeed);

    //System.out.println("Speed * turretspeed     "+speed*turretSpeed);
  }

  public void overRideLimit() {
    int fred = 0;
  }
  public void turretstartzeroing() {
    int fred = 0;
  }
  public double aimMe() {
    int fred = 0;
    return 0.;
  }
  public void stopAimer() {
    int fred = 0;
  }

  /** 
   * if necessary find the turret 180 position
   * limit turret motion
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Move elevator motor down till it finds zero

  }

  void alertuser(boolean alert) {
    if(alert){
     SmartDashboard.putNumber("Turret Alert", 1.);
    } else{
      SmartDashboard.putNumber("Turret Alert", 0.);}
     badposition = alert;
  }

  public void reportLimits() {
    //System.out.println(turretlowerlimit + " < " + turretencoder.getPosition() + " < " + turretupperlimit );
        
  }
}
