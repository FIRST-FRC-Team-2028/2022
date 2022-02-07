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
import frc.robot.QuadraticFitter;
import frc.robot.Pixy2API.Pixy2;
import frc.robot.Pixy2API.links.I2CLink;

/** Turret points toward the hub and shoot balls into it.
 */
public class Turret extends SubsystemBase {
  CANSparkMax turretMotor;
  CANSparkMax elevationMotor;
  RelativeEncoder elevatorencoder;
  boolean elevator_zeroed = false;
  double elevator_zero_position;
  SparkMaxLimitSwitch limitswitch;
  SparkMaxPIDController elevator_controller;
  double elevator_kp=.1;
  double elevator_ki=0.;
  double elevator_kd=0.;
  double elevator_tolerance=2.;  // encoder counts


  CANSparkMax shooter;
  SparkMaxPIDController shooter_controller;
  RelativeEncoder shooterSpeed;
  double shooter_kp=0.1;
  double shooter_ki=0.;
  double shooter_kd=0.;
  double shooter_tolerance=20.;  // RPM
  double distance;

  AnalogInput pixyCam;
  Pixy2 pixyCamI2C;
  PIDController aimer;
  double kp=.3;
  double ki=0.;
  double kd=0.;
  final QuadraticFitter fittere;
  final QuadraticFitter fitterm;

  public Turret() {
    shooter = new CANSparkMax(Constants.CANIDs.TURRET_SHOOTER.getid(), MotorType.kBrushless);
    shooter_controller = shooter.getPIDController();
    shooter_controller.setP(shooter_kp);
    shooter_controller.setI(shooter_ki);
    shooter_controller.setD(shooter_kd);
    turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_AZIMUTH.getid(), MotorType.kBrushless);
    elevationMotor = new CANSparkMax(Constants.CANIDs.TURRET_ELEVATION.getid(), MotorType.kBrushless);
    limitswitch = elevationMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    elevator_controller = elevationMotor.getPIDController();
    elevator_controller.setP(elevator_kp);
    elevator_controller.setI(elevator_ki);
    elevator_controller.setD(elevator_kd);
    elevatorencoder =  elevationMotor.getEncoder();
    /* pixy on I2C or analog?
       Analog is simpler; train for one signature; report x location of largest
       I2C can return more info, ie width, Y  which indicate distance
     */
    pixyCam = new AnalogInput(Constants.TURRET_PIXY_ANALOG);  // to detect hub
    pixyCamI2C = Pixy2.createInstance(new I2CLink());
    int initError = pixyCamI2C.init(Constants.PIXY_USE_MXP);
    aimer = new PIDController(kp, ki, kd);
    aimer.setSetpoint(0.);
    aimer.setIntegratorRange(-1., 1.);
    /** 2d arrays describe elavation and motor speed as functions of distance */
    double[][] motorspeed = {{3.,4.,8.,16.}, {.4,.5,.8,1.}};
    double[][] elavation = {{3.,4.,8.,16.}, {100,200,400,500}};

    fittere =new QuadraticFitter();
    fitterm =new  QuadraticFitter();
 
   
    for(int i=0; i < 4; i++) {
      fittere.add(elavation[0][i], elavation[1][i]);
      fitterm.add(motorspeed[0][i], motorspeed[1][i]);
    }

    limitswitch.enableLimitSwitch(false);  // do not shut down elevator
  
  }

  /** for testing set some arbitrary -1 < value <1 */
  public void shooterOn() {
    shooter.set(Constants.SHOOTER_SPEED);
  }
  /** for closed loop testing set a value -4000 < speed < 4000 (max rpm is 5700) */
  public void shooterSpeed(double speed) {
    shooter_controller.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }
  
  public void shooterOff() {
    shooter.set(0.);
  }

  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  double mapRPM(double distance) {
    return (Constants.SHOOTER_FCN_ACOEF*distance*distance + Constants.SHOOTER_FCN_BCOEF)*distance+Constants.SHOOTER_FCN_CCOEF;
  }
  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  double mapElevation(double distance) {
    return (Constants.ELEVATOR_FCN_ACOEF*distance*distance + Constants.ELEVATOR_FCN_BCOEF)*distance+Constants.ELEVATOR_FCN_CCOEF;
  }
  
  public boolean isatSpeed() {
    //return Math.abs(shooter.get() - fitterm.yout(distance)) < shooter_tolerance;
    return Math.abs(shooter.get() - mapRPM(distance)) < shooter_tolerance;
  }

/**given distance, run motor at set speed */
  public void shooterdistance() {
    //shooter_controller.setReference(fitterm.yout(distance), CANSparkMax.ControlType.kVelocity);
    shooter_controller.setReference(mapRPM(distance), CANSparkMax.ControlType.kVelocity);
  }
  
  public void setelevation() {
    //elevator_controller.setReference(fittere.yout(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
    elevator_controller.setReference(mapElevation(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
  }
  public boolean iselevatordistance() {
    //return Math.abs( elevatorencoder.getPosition() - fittere.yout(distance) + elevator_zero_position) < elevator_tolerance;
    return Math.abs( elevatorencoder.getPosition() - mapElevation(distance) + elevator_zero_position) < elevator_tolerance;
  }
  public void setelevation(double angle) {
    elevator_controller.setReference(angle*Constants.ELEVATOR_ENCODER_RATIO + elevator_zero_position, CANSparkMax.ControlType.kPosition);
  }


  public void setdistance(double distance) {
    this.distance = distance;
  }


  /** Gets rid of opponents ball */
  public void dispose()
  {
    shooter.set(Constants.SHOOTER_SLOW_SPEED);
  }

  /** Use PID controller to aim turret based on pixy camera data.
   * @return error in pixels relative to image width of 315
   */
  public double aimMe() {
    /* connect the camera as a driver for the motors to find the hub */
    double error = pixyCam.getValue();
    //width height i2c LOOK IN DRIVESUBSYSTEM TO GET BIGGEST X AND Y in periodic
    double pidVal = aimer.calculate(error, Constants.CENTER_OF_CAMERA)/ Constants.CENTER_OF_CAMERA;  // pixels/pixels = O(1)
    turretMotor.set(pidVal);
    SmartDashboard.putNumber("Turret Aim Error", Constants.CENTER_OF_CAMERA-error);
    return Constants.CENTER_OF_CAMERA - error;
  }

  public void stopAimer()
  {
    turretMotor.set(0.);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Move elevator motor down till it finds zero
    if( !elevator_zeroed ) {
      elevationMotor.set(Constants.ELEVATOR_MOTOR_ZEROING_SPEED);
      if(limitswitch.isPressed()){
        elevationMotor.set(0.); 
        elevator_zero_position =elevatorencoder.getPosition();
        elevator_zeroed=true;
      }
    }
  }
}
