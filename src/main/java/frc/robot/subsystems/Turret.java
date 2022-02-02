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

public class Turret extends SubsystemBase {
  private static final String SparkMaxRelativeEncoder = null;
  /** Creates a new Turret
   *   to point toward the hub and shoot balls into it. */
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
  double elevator_tolerance=2.;


  CANSparkMax shooter;
  SparkMaxPIDController shooter_controller;
  RelativeEncoder shooterSpeed;
  double shooter_kp=0.1;
  double shooter_ki=0.;
  double shooter_kd=0.;
  double shooter_tolerance=2.;
  double distance;

  AnalogInput pixyCam;
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
    pixyCam = new AnalogInput(Constants.TURRET_PIXY_ANALOG);
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

    limitswitch.enableLimitSwitch(false);
  
  }

  public void shooterOn() {
    shooter.set(Constants.SHOOTER_SPEED);
  }
  public void shooterSpeed(double distance) {
    //shooter.setReference(mapRPM(distance), CANSparkMax.ControlType.kVelocity,)
  }

  public void shooterOff() {
    shooter.set(0.);
  }

  /** Use quadratic curve fit for rpm(distance)  = a x^2 +b x + c */
  double mapRPM(double distance) {
    return (Constants.SHOOTER_FCN_ACOEF*distance*distance + Constants.SHOOTER_FCN_BCOEF)*distance+Constants.SHOOTER_FCN_CCOEF;
  }
  
  public boolean isatSpeed() {
    return Math.abs(shooter.get() - fitterm.yout(distance)) < shooter_tolerance;
  }

/**given distance run motor at set speed */
  public void shooterdistance() {
    shooter_controller.setReference(fitterm.yout(distance), CANSparkMax.ControlType.kVelocity);
  }
  
  public void setelevation() {
    elevator_controller.setReference(fittere.yout(distance) + elevator_zero_position, CANSparkMax.ControlType.kPosition);
  }
  public boolean iselevatordistance() {
    return Math.abs( elevatorencoder.getPosition() - fittere.yout(distance) + elevator_zero_position) < elevator_tolerance;
  }
  public void setdistance(double distance) {
    this.distance = distance;
  }


  /** Gets rid of opponents ball */
  public void dispose()
  {
    shooter.set(Constants.SHOOTER_SLOW_SPEED);
  }

  public double aimMe() {
    /** connenct the camera as a driver for the motors to find the hub */
    double error = pixyCam.getValue();
    double pidVal = aimer.calculate(error, 0.);
    turretMotor.set(pidVal);
    SmartDashboard.putNumber("Turret Aim Error", error);
    return error;
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
