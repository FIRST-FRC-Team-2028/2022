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
 *     shooter:
 *        shooterOn
 *        hasShot
 *        shooterSpeed
 *        shooterOff
 *        isatSpeed
 *        shooterDistance
 * 
 *     elevator:
 */

/** Turret points toward the hub and shoot balls into it.
 */
public class Shooter extends SubsystemBase {
  
  CANSparkMax elevationMotor;
  RelativeEncoder elevatorencoder;
  boolean elevator_zeroed = true;
  double FIX_ME = 1.;  //TODO:FIX Elevator zeroed
  double elevator_zero_position;
  SparkMaxLimitSwitch limitswitch;
  SparkMaxPIDController elevator_controller;
  double elevator_kp=.1;
  double elevator_ki=0.;
  double elevator_kd=0.;
  double elevator_tolerance=2.;  // encoder counts
  //AnalogInput turretswitch;
  SparkMaxLimitSwitch turretswitch;
  int zeroiter = 0;

  CANSparkMax shooter;
  SparkMaxPIDController shooter_controller;
  RelativeEncoder shooterSpeed;
  RobotContainer robotContainer;
  static final int NUM_ENC = 50;
  double[] encoder_velocity = new double[NUM_ENC]; 
  int iter = 0;
  double enc_avg;
  double shooter_kp=5.e-4;
  double shooter_ki=1.e-6;
  double shooter_kd=1.;
  double shooterkMaxout = 1.;
  double shooterkMinout = -1.;
  double shooter_tolerance=20.;  // RPM
  double distance;

  AnalogInput pixyCam;
  Pixy2 pixyCamI2C;
  PIDController aimer;
  double kp=.3;
  double ki=0.;
  double kd=0.;
  /*
  final QuadraticFitter fittere;
  final QuadraticFitter fitterm;
  */

  public Shooter(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    if (Constants.SHOOTER_AVAILABLE){

      shooter = new CANSparkMax(Constants.CANIDs.TURRET_SHOOTER.getid(), MotorType.kBrushless);
      shooter.restoreFactoryDefaults();
      shooter.setInverted(Constants.CANIDs.TURRET_SHOOTER.isInverted());
      shooter_controller = shooter.getPIDController();
      shooter_controller.setP(shooter_kp);
      shooter_controller.setI(shooter_ki);
      shooter_controller.setD(shooter_kd);
      shooterSpeed = shooter.getEncoder();
    }
    
    if (Constants.ELEVATOR_AVAILABLE){
      elevationMotor = new CANSparkMax(Constants.CANIDs.TURRET_ELEVATION.getid(), MotorType.kBrushless);
      elevationMotor.restoreFactoryDefaults();
      elevationMotor.setInverted(Constants.CANIDs.TURRET_ELEVATION.isInverted());
      elevationMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      limitswitch = elevationMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      elevator_controller = elevationMotor.getPIDController();
      elevator_controller.setP(elevator_kp);
      elevator_controller.setI(elevator_ki);
      elevator_controller.setD(elevator_kd);
      elevatorencoder =  elevationMotor.getEncoder();
      limitswitch.enableLimitSwitch(false);  // do not shut down elevator
    }
    
  
    

    //SmartDashboard.putNumber("Turret Alert", 0.);  // Tells driver that turret is located in a safe region

    for(int i = 0; i < NUM_ENC; i++ ) encoder_velocity[i] = 0; 
    enc_avg = 0.;

    /** 2d arrays describe elavation and motor speed as functions of distance */
    /*
    double[][] motorspeed = {{3.,4.,8.,16.}, {.4,.5,.8,1.}};
    double[][] elavation = {{3.,4.,8.,16.}, {100,200,400,500}};

    fittere =new QuadraticFitter();
    fitterm =new  QuadraticFitter();
 
    for(int i=0; i < 4; i++) {
      fittere.add(elavation[0][i], elavation[1][i]);
      fitterm.add(motorspeed[0][i], motorspeed[1][i]);
    }
    */

  }

  boolean shooteron;
  private boolean engagedcargo;
  private boolean engagedLow;
  private boolean dontCount;
  /** for testing set some arbitrary -1 < value <1 
  public void shooterOn() {
    shooter.set(Constants.SHOOTER_SPEED);
    shooteron = true;
  }
  */

  /** determine whether cargo has been shot
   * based on variations of RPM from the steady state
    */
  public boolean hasShot() {
    // look for variation
    double delta;
    if (!dontCount) {
      delta = shooterSpeed.getVelocity() - enc_avg;
      if (!engagedcargo && delta >/*500*/ Constants.SHOOT_INDICATORUP) {
        engagedcargo = true;
        System.out.println("Saw the blip up "+delta);
        return false;
      }
      if(engagedcargo && delta < /*-300*/Constants.SHOOT_INDICATORDOWN) {
        engagedLow = true;
        System.out.println("Saw the blip down "+delta);
        return false;
      }
      if (engagedcargo && engagedLow && Math.abs(delta) <100) {
        engagedLow = false;
        engagedcargo = false;
        System.out.println("Detected back to avg");
        return true;
      }
    }
    return false;
  }

  /** for closed loop testing set a value -4000 < speed < 4000 (max rpm is 5700) */
  public void shooterSpeed(double speed) {
    shooter_controller.setReference(speed, CANSparkMax.ControlType.kVelocity);
    for(int i = 0; i < NUM_ENC; i++) encoder_velocity[i] = speed;
    enc_avg = speed;
    iter = 0;
  }
  
  public void shooterOff() {
    shooter.set(0.);
    for(int i = 0; i < NUM_ENC; i++) encoder_velocity[i] = 0.;
    enc_avg = 0.;
    iter = 0;
    shooteron = false;
    dontCount = true; // wait for steadyState before looking for evidence of shooting
    dontCounter = 0;
  }

  /** Gets rid of opponents ball 
  public void dispose()
  {
    shooter.set(Constants.SHOOTER_SLOW_SPEED);
  }
  */

  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  /* Use discrete vlues from test
     tarmac: 2300 low, 2900 high; both went in
     cargo ring:  2900 low, 3250 high; both went in
     pad1: untested
     dribble or low hub: 1300
     */
  double mapRPM(double distance) {
    //return (Constants.SHOOTER_FCN_ACOEF*distance*distance + Constants.SHOOTER_FCN_BCOEF)*distance+Constants.SHOOTER_FCN_CCOEF;
    if (distance >= Constants.FIELD_HUB_TO_PAD1) return Constants.PAD_ONE_RPM;
    if (distance >= Constants.CARGO_RING_DISTANCE) return Constants.CARGO_RING_RPM;
    if (distance >= Constants.TARMAC_DISTANCE) return Constants.TARMAC_RPM;
    return Constants.DRIBBLE_RPM;
  }
  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  double mapElevation(double distance) {
    return (Constants.ELEVATOR_FCN_ACOEF*distance*distance + Constants.ELEVATOR_FCN_BCOEF)*distance+Constants.ELEVATOR_FCN_CCOEF;
  }
  
  /** report whether shooter has reached desired RPM */
  public boolean isatSpeed() {
    //double desired = fitterm.yout(distance);
    double desired = mapRPM(distance);
    //System.out.println(shooterSpeed.getVelocity() + "-" + desired + "<" + shooter_tolerance);
    return Math.abs(shooterSpeed.getVelocity() - desired) < shooter_tolerance;
  }

  private int dontCountRange;
  /**given distance, run motor at set speed */
  public void shooterdistance() {
    //double desired = fitterm.yout(distance);
    double desired = mapRPM(distance);
    shooter_controller.setReference(desired, CANSparkMax.ControlType.kVelocity);
    shooteron = true;
    dontCountRange=200;
    if (desired > 2000.) dontCountRange = 330;
    if (desired > 2500.) dontCountRange = 450;
    if (desired > 3000.) dontCountRange = 650;
  }
  
  public void setelevation() {
    if (Constants.ELEVATOR_AVAILABLE) {
    //elevator_controller.setReference(fittere.yout(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
    elevator_controller.setReference(mapElevation(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
    }
  }

  /**  report whether elevator has reached set point
   * @return boolean
  */
  public boolean iselevatordistance() {
    if (Constants.ELEVATOR_AVAILABLE) {
      //return Math.abs( elevatorencoder.getPosition() - fittere.yout(distance) + elevator_zero_position) < elevator_tolerance;
      return Math.abs( elevatorencoder.getPosition() - mapElevation(distance) + elevator_zero_position) < elevator_tolerance;
    }
    return true;
  }
  public void setelevation(double angle) {
    if (Constants.ELEVATOR_AVAILABLE) {
      elevator_controller.setReference(angle*Constants.ELEVATOR_ENCODER_RATIO + elevator_zero_position, CANSparkMax.ControlType.kPosition);
    }
  }

  /** set speed of elevation motor
   * @param speed
   * For testing and calibration - not for typical operation
   */
  public void elevatorMove(double speed) {
    elevationMotor.set(speed);
  }
  public double getElevation() {
    return elevatorencoder.getPosition();
  }


  public void setdistance(double distance) {
    this.distance = distance;
  }

  public double aimMe() {
    int fred = 0;
    return 0.;
  }
  public void stopAimer() {
    int fred = 0;
  }
  

  private int dontCounter;
  
  /** 
   * if necessary, find the elevator zero
   * if necessary find the turret 180 position
   * limit turret motion
   * compute shooter RPM running average
   * allow shooter to reach speed before shooting is observed
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Move elevator motor down till it finds zero
    if(Constants.ELEVATOR_AVAILABLE) {
      if( !elevator_zeroed ) {
        elevationMotor.set(Constants.ELEVATOR_MOTOR_ZEROING_SPEED);
        if(limitswitch.isPressed()){
          elevationMotor.set(0.); 
          elevator_zero_position =elevatorencoder.getPosition();
          elevator_zeroed=true;
        }
      }
    } 

    if (Constants.SHOOTER_AVAILABLE) {
      if(shooteron) {
        // based on experiment, it takes a number of iterations to stabilize the RPM from zero
        if (dontCount) {  
          dontCounter +=1;
          if (dontCounter > dontCountRange) {
            dontCount = false;
          }
        }else {
          enc_avg = enc_avg  - encoder_velocity[iter]/NUM_ENC;
          encoder_velocity[iter] = shooterSpeed.getVelocity();
          iter = (iter+1)%NUM_ENC;
          enc_avg = enc_avg  + shooterSpeed.getVelocity()/NUM_ENC;
        }
      }
    }
  }

}
