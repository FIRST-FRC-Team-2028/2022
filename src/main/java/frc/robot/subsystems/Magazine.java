// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Transfers ammo from pickup to shooter */
public class Magazine extends SubsystemBase {
  private CANSparkMax horizontalmotor;
  private CANSparkMax verticalmotor;
  /** Creates a new Magazine.
   * Uses two seperate motors to transport ammo horizontally and vertically
   */
  public Magazine() {
    horizontalmotor= new CANSparkMax(Constants.CANIDs.MAGIZINE_HORIZONTAL.getid(), MotorType.kBrushless);
    verticalmotor= new CANSparkMax(Constants.CANIDs.MAGIZINE_VERTICAL.getid(), MotorType.kBrushless);
  }
  
  /**moves cargo along horizontal conveyor */
  public void horizontalon() {
    horizontalmotor.set(Constants.MAGIZINE_HORIZONTAL_MOTOR_SPEED);
  }

  /**moves cargo along vertical conveyor */
  public void verticalon() {
    verticalmotor.set(Constants.MAGIZINE_VERTICAL_MOTOR_SPEED);
  }

  public void horizontaloff() {
    horizontalmotor.set(0.);
  }

  public void verticaloff() {
    verticalmotor.set(0.);
  }

  

  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
