// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends SubsystemBase {
  CANSparkMax climbMotor;
  double starttime;
  RelativeEncoder encoder;
  SparkMaxPIDController m_pidController;
  double kP;
  double kI;
  double kD;
  private double kIz;
  private double kFF;
  private double kMaxOutput;
  private double kMinOutput;
  
  
  /** Creates a new Climber. */
  public Climber() {
    climbMotor = new CANSparkMax(Constants.CANIDs.CLIMB_MOTOR.getid(), MotorType.kBrushless);
    climbMotor.restoreFactoryDefaults();
    climbMotor.setInverted(Constants.CANIDs.CLIMB_MOTOR.isInverted());
    double I_NEED_WORK = 0.;  // There is a motor for each climber arm; both need controlled
    
    encoder = climbMotor.getEncoder();
    m_pidController = climbMotor.getPIDController();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    encoder.setPositionConversionFactor(Constants.CLIMBER_GEAR_RATIO);
    encoder.setPosition(0.);
    

  }

  /** deploys climber to specified height  */
  public void deployclimber() {
    
    m_pidController.setReference( Constants.HANGAR_BAR_HEIGHT - Constants.ROBOT_CLIMBER_STARTING_HEIGHT , CANSparkMax.ControlType.kPosition);
  }

  /** pulls robot up to the bar */
  public void pullup() {
    m_pidController.setReference(Constants.HANGAR_BAR_HEIGHT - Constants.HEIGHT_TO_CLIMB, CANSparkMax.ControlType.kPosition);
  }

  /** determine if climber has reached its destination */
  public boolean amidone() {
    double I_NEED_WORK = 666.; // it must eventually be true
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
