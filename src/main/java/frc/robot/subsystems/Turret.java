// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret
   *   to point toward the hub and shoot balls into it. */
  CANSparkMax turretMotor;
  CANSparkMax elevationMotor;
  CANSparkMax shooter;
  AnalogInput pixyCam;
  PIDController aimer;
  double kp=.3;
  double ki=0.;
  double kd=0.;
  public Turret() {
    shooter = new CANSparkMax(Constants.CANIDs.TURRET_SHOOTER.getid(), MotorType.kBrushless);
    turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_AZIMUTH.getid(), MotorType.kBrushless);
    elevationMotor = new CANSparkMax(Constants.CANIDs.TURRET_ELEVATION.getid(), MotorType.kBrushless);
    pixyCam = new AnalogInput(Constants.TURRET_PIXY_ANALOG);
    aimer = new PIDController(kp, ki, kd);
    aimer.setSetpoint(0.);
    aimer.setIntegratorRange(-1., 1.);
  }

  public void shooterOn() {
    shooter.set(Constants.SHOOTER_SPEED);
  }

  public void shooterOff() {
    shooter.set(0.);
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
