// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Pixy2API.Pixy2;
import frc.robot.Pixy2API.links.I2CLink;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveTowardBall;
import frc.robot.commands.PickupTargets;
import frc.robot.commands.StopMotor;
import frc.robot.commands.TurnoffPickup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Joystick m_joystick = new Joystick(Constants.JOYSTICK);
  

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem;
  private final Pickup pickup;
 // private final Turret turret;
  private final Pixy2 driveCamera;

  private final DefaultDriveCommand m_defaultDriveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Configure default commands
    //if (Constants.DRIVE_AVAILABLE) {
      m_driveSubsystem = new DriveSubsystem();
      m_defaultDriveCommand = new DefaultDriveCommand(m_driveSubsystem, m_joystick);
      m_driveSubsystem.setDefaultCommand(m_defaultDriveCommand);
    //}

    //if (Constants.PICKUP_AVAILABLE) {
      pickup = new Pickup();
    //}

    //if (Constants.TURRET_AVAILABLE) {
    //  turret = new Turret();
    //}

    driveCamera = Pixy2.createInstance(new I2CLink());
    int initError = driveCamera.init(1);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton shifter = new JoystickButton(m_joystick,Constants.SHIFTER_BUTTON);

    JoystickButton pickupDeployer = new JoystickButton(m_joystick, Constants.DEPLOY_PICKUP_BUTTON);
    JoystickButton pickupUnDeployer = new JoystickButton(m_joystick, Constants.RETRACT_PICKUP_BUTTON);
    pickupDeployer.whenPressed(new PickupTargets(pickup));
    pickupUnDeployer.whenPressed(new TurnoffPickup(pickup));
    
    JoystickButton driveToBall = new JoystickButton(m_joystick, Constants.DRIVE_TO_BALL_BUTTON);
    driveToBall.whenPressed(new DriveTowardBall(m_driveSubsystem, driveCamera, m_joystick));
    driveToBall.whenReleased(new StopMotor(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
