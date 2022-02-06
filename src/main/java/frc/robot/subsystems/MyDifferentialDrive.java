// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** MyDifferentialDrive provides an alternative arcadeDrive method
 *  that implements closed loop velocity control to drive the robot.
 *  It extends DifferentialDrive to use all the other functionality,
 *  ie deadband, normalize
 */
public class MyDifferentialDrive extends DifferentialDrive {
    private final MotorController m_leftMotor;
    private final MotorController m_rightMotor;
    private final SparkMaxPIDController m_leftController;
    private final SparkMaxPIDController m_rightController;
    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;
    
    public MyDifferentialDrive(MotorController leftMotor, MotorController rightMotor
    , SparkMaxPIDController leftController, SparkMaxPIDController righController
    , RelativeEncoder leftEncoder, RelativeEncoder rightEncoder
    ) {
        super(leftMotor, rightMotor);
        m_leftMotor = leftMotor;
        m_rightMotor= rightMotor;
        m_leftController = leftController;
        m_rightController = righController;
        m_rightEncoder = rightEncoder;
        m_leftEncoder = leftEncoder;
    }
    
    
    /**
   * Arcade drive method for differential drive platform. The calculated values will be squared to
   * decrease sensitivity at low speeds.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   */
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation) {
    arcadeDrive(xSpeed, zRotation, true);
  }

  /**
   * Arcade drive method for differential drive platform.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   * Overload the WPIlib version to control the motor speed via closed loop controller
   */
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    /*f (!m_reported) {
      HAL.report(
          tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_DifferentialArcade, 2);
      m_reported = true;
    } */

    xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
    zRotation = MathUtil.applyDeadband(zRotation, m_deadband);

    var speeds = arcadeDriveIK(xSpeed, zRotation, squareInputs);

    if (Constants.DRIVE_VELOCITY_CONTROLLED) {
      /* with closed loop control */
      SmartDashboard.putNumber("myArcade left", speeds.left);
      SmartDashboard.putNumber("myArcade right", speeds.right);
      m_leftController.setReference(speeds.left * Constants.SPARKMAX_RPM, CANSparkMax.ControlType.kVelocity);
      m_rightController.setReference(speeds.right * Constants.SPARKMAX_RPM, CANSparkMax.ControlType.kVelocity);
    } else {
      /* replace direct drive */
      m_leftMotor.set(speeds.left * m_maxOutput);
      m_rightMotor.set(speeds.right * m_maxOutput);
    }
    
    SmartDashboard.putNumber("leftRPM",m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("rightRPM",m_rightEncoder.getVelocity());
    feed();
  }
}
