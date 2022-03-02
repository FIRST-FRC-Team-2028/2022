// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
 * 
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

  CANSparkMax shooter;
  SparkMaxPIDController shooter_controller;
  RelativeEncoder shooterSpeed;
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

  public Turret() {
    if (Constants.SHOOTER_AVAILABLE){

      shooter = new CANSparkMax(Constants.CANIDs.TURRET_SHOOTER.getid(), MotorType.kBrushless);
      shooter.setInverted(Constants.CANIDs.TURRET_SHOOTER.isInverted());
      shooter_controller = shooter.getPIDController();
      shooter_controller.setP(shooter_kp);
      shooter_controller.setI(shooter_ki);
      shooter_controller.setD(shooter_kd);
    }
    if (Constants.TURRET_AVAILABLE){

      turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_AZIMUTH.getid(), MotorType.kBrushless);
      turretencoder =  turretMotor.getEncoder();
      //turretswitch = new AnalogInput(Constants.TURRET_SWITCH_CHANNEL);
      turretswitch = turretMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    }
    if (Constants.ELEVATOR_AVAILABLE){
      elevationMotor = new CANSparkMax(Constants.CANIDs.TURRET_ELEVATION.getid(), MotorType.kBrushless);
      limitswitch = elevationMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      elevator_controller = elevationMotor.getPIDController();
      elevator_controller.setP(elevator_kp);
      elevator_controller.setI(elevator_ki);
      elevator_controller.setD(elevator_kd);
      elevatorencoder =  elevationMotor.getEncoder();
      limitswitch.enableLimitSwitch(false);  // do not shut down elevator
    }
    
  
    /* pixy on I2C or analog?
       Analog is simpler; train for one signature; report x location of largest
       I2C can return more info, ie width, Y  which indicate distance
     */
    pixyCam = new AnalogInput(Constants.TURRET_PIXY_ANALOG);  // to detect hub
    if (Constants.I2C_CAM_AVAILABLE) {
      pixyCamI2C = Pixy2.createInstance(new I2CLink());
      int initError = pixyCamI2C.init(Constants.PIXY_USE_MXP, Constants.TURRET_PIXY_ADDRESS);
      if(initError != 0 ){
        System.out.println("pixy did not intitialize " + initError);
      }
    }
    aimer = new PIDController(kp, ki, kd);
    aimer.setSetpoint(0.);
    aimer.setIntegratorRange(-1., 1.);

    SmartDashboard.putNumber("Turret Alert", 0.);  // Tells driver that turret is located in a safe region

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
  /** for testing set some arbitrary -1 < value <1 */
  public void shooterOn() {
    shooter.set(Constants.SHOOTER_SPEED);
    shooteron = true;
  }

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

  /** Gets rid of opponents ball */
  public void dispose()
  {
    shooter.set(Constants.SHOOTER_SLOW_SPEED);
  }

  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  double mapRPM(double distance) {
    return (Constants.SHOOTER_FCN_ACOEF*distance*distance + Constants.SHOOTER_FCN_BCOEF)*distance+Constants.SHOOTER_FCN_CCOEF;
  }
  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  double mapElevation(double distance) {
    return (Constants.ELEVATOR_FCN_ACOEF*distance*distance + Constants.ELEVATOR_FCN_BCOEF)*distance+Constants.ELEVATOR_FCN_CCOEF;
  }
  
  /** report whether shooter has reached desired RPM */
  public boolean isatSpeed() {
    //double desired = fitterm.yout(distance);
    double desired = mapRPM(distance);
    return Math.abs(shooter.get() - desired) < shooter_tolerance;
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
    //elevator_controller.setReference(fittere.yout(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
    elevator_controller.setReference(mapElevation(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
  }

  /**  report whether elevator has reached set point
   * @return boolean
  */
  public boolean iselevatordistance() {
    //return Math.abs( elevatorencoder.getPosition() - fittere.yout(distance) + elevator_zero_position) < elevator_tolerance;
    return Math.abs( elevatorencoder.getPosition() - mapElevation(distance) + elevator_zero_position) < elevator_tolerance;
  }
  public void setelevation(double angle) {
    elevator_controller.setReference(angle*Constants.ELEVATOR_ENCODER_RATIO + elevator_zero_position, CANSparkMax.ControlType.kPosition);
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


  /** Use PID controller to aim turret based on pixy camera data.
   * @return error in pixels relative to image width of 315
   */
  public double aimMe() {
    /* connect the camera as a driver for the motors to find the hub */
    //double error = pixyCam.getValue();
    if (Constants.I2C_CAM_AVAILABLE) {
      int numTargets = pixyCamI2C.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL,10);
      if(numTargets == 0) return 0.;    
      Block biggest=null;
      double size=0.;
      double filterSize= Constants.TURRET_AIMER_FILTER_SIZE;
      for (Block block : pixyCamI2C.getCCC().getBlockCache())  {
         if (block.getWidth() < filterSize) {
           block.getSignature();
           block.getWidth();
           block.getHeight();
           
           double tsize = block.getWidth()*block.getHeight();
           if (tsize > size && block.getSignature() == Constants.PIXY_SIG_HUB) {
             size = tsize;
             biggest = block;
           }
         }
      }
  
      //get distance from pixy to object, assuming pixy aiming level
      double tana = ((Constants.PIXY_VERT_CENTER - biggest.getY()) / Constants.PIXY_VERT_CENTER)
                      *Constants.PIXY_TAN_VERT_FOV;
      double tanApG = (tana + Constants.PIXY_GAM_TURRET_CAM_ANGLE)/(1.-tana*Constants.PIXY_GAM_TURRET_CAM_ANGLE);
      distance = (Constants.HUB_HEIGHT - Constants.TURRET_CAM_HEIGHT)
                  / tanApG;
  
      // use PIDcontroller to aim turret
      double measure = biggest.getX();
      if ( measure < 0.) return 0.;
      double pidVal = aimer.calculate(measure, Constants.CENTER_OF_CAMERA)/ Constants.CENTER_OF_CAMERA;  // pixels/pixels = O(1)
      turretMotor.set(pidVal);
      SmartDashboard.putNumber("Turret Aim Error", Constants.CENTER_OF_CAMERA-measure);
      return Constants.CENTER_OF_CAMERA - measure;   
    } else return 0.;


    ///Constants.CENTER_OF_CAMERA));
  }

  public void stopAimer()
  {
    turretMotor.set(0.);
  }

  public void turretFine(boolean fine) {
    if (fine) turretSpeed = Constants.TURRET_MOTOR_SLOW_SPEED;
    else turretSpeed = Constants.TURRET_MOTOR_SPEED;
  }
  /** move turret
   * @param speed [+1, 0., -1] positive clockwise around up vector
   */
  public void turretCW(double speed){
    turretMotor.set(speed*turretSpeed);
  }

  public void turretstartzeroing() {
     turretiszeroed = false;
  }
  
  boolean AMIRIGHT = true;
  /**
   * Set side of robot turret is facing at match start
   * @param right 
   */
  public void setTurretStartSector(boolean right) {
    AMIRIGHT = right;
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

    if (Constants.TURRET_AVAILABLE){
      // Presuming turret is 
      if(!turretiszeroed) {
        turretMotor.set((AMIRIGHT?1.:-1.)*Constants.TURRET_MOTOR_ZEROING_SPEED);
        double NEEDS_WORK = 1.e5;
        // mr.g says look at this to determine direction
        // and search limit TODO
        if(limitswitch.isPressed()) {
          turretMotor.set(0.);
          double turret_zero_position = turretencoder.getPosition(); 
          turretupperlimit = turret_zero_position + 
                (360.*(AMIRIGHT?0.:1.) + 20.)*Constants.TURRET_ENCODER_RATIO; 
          turretlowerlimit = turret_zero_position - 
                (360.*(AMIRIGHT?1.:0) + 20.)*Constants.TURRET_ENCODER_RATIO;
          turretiszeroed = true;
        }
      } 
      if(turretencoder.getPosition() > turretupperlimit ) {
        turretMotor.set(Math.min(0., turretMotor.get()));
      } 
  
      if(turretencoder.getPosition() < turretlowerlimit ) {
        turretMotor.set(Math.max(0., turretMotor.get()));
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

  void alertuser(boolean alert) {
    if(alert){
     SmartDashboard.putNumber("Turret Alert", 1.);
    } else{
      SmartDashboard.putNumber("Turret Alert", 0.);}
     badposition = alert;
  }
}
