// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Transfers cargo from pickup to shooter */
public class Magazine extends SubsystemBase {
  private CANSparkMax horizontalmotor;
  private CANSparkMax verticalmotor;
  
  /** Uses two separate motors to transport cargo horizontally and vertically
   */
  public Magazine() {
    //horizontalmotor= new CANSparkMax(Constants.CANIDs.MAGIZINE_HORIZONTAL.getid(), MotorType.kBrushless);
    verticalmotor= new CANSparkMax(Constants.CANIDs.MAGIZINE_VERTICAL.getid(), MotorType.kBrushless);
    verticalmotor.restoreFactoryDefaults();
    verticalmotor.setInverted(Constants.CANIDs.MAGIZINE_VERTICAL.isInverted());
  }
  
  /**moves cargo along horizontal conveyor */
  public void horizontalon() {
    //horizontalmotor.set(Constants.MAGAZINE_HORIZONTAL_MOTOR_SPEED);
  }

  /**moves cargo along vertical conveyor */
  public void verticalon() {
    verticalon(1.);
  }
  public void verticalon(double direction) {
    verticalmotor.set(direction*Constants.MAGAZINE_VERTICAL_MOTOR_SPEED);
  }

  public void horizontaloff() {
   // horizontalmotor.set(0.);
  }

  public void verticaloff() {
    verticalmotor.set(0.);
  }

  

  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
