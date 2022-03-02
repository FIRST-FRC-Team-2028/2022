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

public class Climber extends SubsystemBase {
  CANSparkMax climbMotor;
  CANSparkMax climbMotor2;
  double starttime;
  RelativeEncoder encoder, encoder2;
  SparkMaxPIDController m_pidController, m_pidController2;
  double kP;
  double kI;
  double kD;
  private double kIz;
  private double kFF;
  private double kMaxOutput;
  private double kMinOutput;
  boolean pullingUp;
  double target;
  double tolerance;
  
  
  /** Creates a new Climber. */
  public Climber() {
    climbMotor = new CANSparkMax(Constants.CANIDs.CLIMB_MOTOR.getid(), MotorType.kBrushless);
    climbMotor.restoreFactoryDefaults();
    climbMotor.setInverted(Constants.CANIDs.CLIMB_MOTOR.isInverted());

    climbMotor2 = new CANSparkMax(Constants.CANIDs.CLIMB_MOTOR2.getid(), MotorType.kBrushless);
    climbMotor2.restoreFactoryDefaults();
    climbMotor2.setInverted(Constants.CANIDs.CLIMB_MOTOR2.isInverted());
    
    encoder = climbMotor.getEncoder();
    m_pidController = climbMotor.getPIDController();

    encoder2 = climbMotor2.getEncoder();
    m_pidController2 = climbMotor2.getPIDController();

    pullingUp = false;
    tolerance = 0.5; //inches

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
    
    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);
    encoder2.setPositionConversionFactor(Constants.CLIMBER_GEAR_RATIO);
    encoder2.setPosition(0.);
  }

  /** deploys climber to specified height  */
  public void deployclimber() {
    m_pidController.setReference(Constants.HANGAR_BAR_HEIGHT - Constants.ROBOT_CLIMBER_STARTING_HEIGHT, CANSparkMax.ControlType.kPosition);
    m_pidController2.setReference(Constants.HANGAR_BAR_HEIGHT - Constants.ROBOT_CLIMBER_STARTING_HEIGHT, CANSparkMax.ControlType.kPosition);
    target = Constants.HANGAR_BAR_HEIGHT - Constants.ROBOT_CLIMBER_STARTING_HEIGHT;
  }

  /** pulls robot up to the bar */
  public void pullup() {
    m_pidController.setReference(Constants.HANGAR_BAR_HEIGHT - Constants.HEIGHT_TO_CLIMB, CANSparkMax.ControlType.kPosition);
    target = Constants.HANGAR_BAR_HEIGHT - Constants.HEIGHT_TO_CLIMB;
    //used in periodic to follow pid1
    //m_pidController2.setReference(Constants.HANGAR_BAR_HEIGHT - Constants.HEIGHT_TO_CLIMB, CANSparkMax.ControlType.kPosition);
    pullingUp = true;
  }

  /** determine if climber has reached its destination */
  public boolean amidone() {
    if (Math.abs(target - encoder.getPosition()) < tolerance) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //after pullup command is called
    //make controller #2 follow #1
    if (pullingUp) {
      m_pidController2.setReference(encoder.getPosition(), CANSparkMax.ControlType.kPosition);
    }
  }
}
