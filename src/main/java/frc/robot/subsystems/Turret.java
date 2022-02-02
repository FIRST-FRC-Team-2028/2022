// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
  CANSparkMax shooter;
  RelativeEncoder shooterSpeed;
  AnalogInput pixyCam;
  PIDController aimer;
  double kp=.3;
  double ki=0.;
  double kd=0.;
  boolean amicallibrated = false;
  final QuadraticFitter fittere;
  final QuadraticFitter fitterm;
  public Turret() {
    shooter = new CANSparkMax(Constants.CANIDs.TURRET_SHOOTER.getid(), MotorType.kBrushless);
    //shooter.something(CANSparkMax.ControlType.kVelocity);
    turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_AZIMUTH.getid(), MotorType.kBrushless);
    elevationMotor = new CANSparkMax(Constants.CANIDs.TURRET_ELEVATION.getid(), MotorType.kBrushless);
    //elevatorencoder =  elevationMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, Constants.ELEVATOR_MOTOR_ENCODER_RATIO);
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

  double mapRPM(double distance) {
    return (Constants.SHOOTER_FCN_ACOEF*distance*distance + Constants.SHOOTER_FCN_BCOEF)*distance+Constants.SHOOTER_FCN_CCOEF;
  }
  
  public boolean isatSpeed(double distance) {
    return (shooter.get() == fitterm.yout(distance));
  }

/**given distance run motor at set speed */
  public void shooterdistance(double distance) {
  shooter.set(fitterm.yout(distance));
  }
  
  public void setelevation(double distance) {
    elevatorencoder.setPosition(fittere.yout(distance));
  }
  public boolean iselevatordistance(double distance) {
    return ( elevatorencoder.getPosition() ==  fittere.yout(distance));
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
  }
}
