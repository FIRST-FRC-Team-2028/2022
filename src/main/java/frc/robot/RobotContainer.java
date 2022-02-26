// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoDriveToCargo;
import frc.robot.commands.AutoLeaveTarmacDistance;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootAndGetCargo;
import frc.robot.commands.AutoShootTwo;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveTowardBall;
import frc.robot.commands.PickupTargets;
import frc.robot.commands.RetractPickup;
import frc.robot.commands.StopMotor;
import frc.robot.commands.TurnoffPickup;
import frc.robot.commands.TurretCCW;
import frc.robot.commands.TurretCW;
import frc.robot.commands.TurretFine;
import frc.robot.commands.TurretStop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DeployClimber;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootStop;
import frc.robot.commands.ShiftGears;
import frc.robot.commands.ShiftGearsU;
import frc.robot.commands.TurnoffPickup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Joystick m_joystick = new Joystick(Constants.JOYSTICK);
  Joystick buttonBoxLeft = new Joystick(Constants.LEFT_BUTTON_BOX);
  Joystick buttonBoxRight = new Joystick(Constants.RIGHT_BUTTON_BOX);
  

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem;

  /* Make final only when available
  private final Pickup pickup;
  private final Turret turret;
  private final Pixy2 driveCamera;
  private final Magazine magazine;
  */
  private Pickup pickup;
  private Turret turret;
  private Magazine magazine;
  
  private PneumaticHub pcm;

  private final DefaultDriveCommand m_defaultDriveCommand;
  SendableChooser<Command> m_chooser;
  SendableChooser<Boolean> sideChooser;
    
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_joystick = new Joystick(Constants.JOYSTICK);
    buttonBoxLeft = new Joystick(Constants.LEFT_BUTTON_BOX);
    buttonBoxRight = new Joystick(Constants.RIGHT_BUTTON_BOX);
    //Configure default commands
    if (Constants.COMPRESSOR_AVAILABLE ){
      pcm = new PneumaticHub(Constants.PNEUMATICS_CONTROL_MODULE);
    }

    if (Constants.DRIVE_AVAILABLE) {
      m_driveSubsystem = new DriveSubsystem(pcm);
      m_defaultDriveCommand = new DefaultDriveCommand(m_driveSubsystem, m_joystick);
      m_driveSubsystem.setDefaultCommand(m_defaultDriveCommand);
    }

    if (Constants.PICKUP_AVAILABLE) {
      pickup = new Pickup(pcm);
    }

    if (Constants.TURRET_AVAILABLE) {
      turret = new Turret();
    }

    if (Constants.MAGAZINE_AVAILABLE) {
      magazine = new Magazine();
    }

  
    // Configure the button bindings
    configureButtonBindings();
  }
  public PneumaticHub getPcm() {
    return pcm;
  }
  public Turret getTurret() {
    return turret;
  }
  public Pickup getPickup() {
    return pickup;
  }
  public Joystick getJoystick() {
    return m_joystick;
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (!Constants.AUTOSHIFT_AVAILABLE) {
      JoystickButton shifter = new JoystickButton(m_joystick,Constants.SHIFTER_BUTTON);
      shifter.whenPressed(new ShiftGears(m_driveSubsystem));
      JoystickButton shifterU = new JoystickButton(m_joystick,Constants.SHIFTERU_BUTTON);
      shifterU.whenPressed(new ShiftGearsU(m_driveSubsystem, Constants.DRIVE_HIGH_GEAR));
      JoystickButton shifterD = new JoystickButton(m_joystick,Constants.SHIFTERD_BUTTON);
      shifterD.whenPressed(new ShiftGearsU(m_driveSubsystem, Constants.DRIVE_LOW_GEAR));
  }
    
    if (Constants.PICKUP_AVAILABLE /*&& Constants.MAGAZINE_AVAILABLE*/) {
      JoystickButton pickupDeployer = new JoystickButton(buttonBoxLeft, Constants.DEPLOY_PICKUP_BUTTON);
      JoystickButton pickupUnDeployer = new JoystickButton(buttonBoxRight, Constants.RETRACT_PICKUP_BUTTON);
      JoystickButton rollerStopper = new JoystickButton(buttonBoxLeft, Constants.ROLLER_STOP_BUTTON);
      pickupDeployer.whenPressed(new PickupTargets(pickup/*,magazine*/));
      //pickupUnDeployer.whenPressed(new TurnoffPickup(pickup/*,magazine*/));
      pickupUnDeployer.whenPressed(new RetractPickup(pickup));
      rollerStopper.whenPressed(new TurnoffPickup(pickup));
    }

    if (Constants.DRIVE_AVAILABLE && Constants.CAMERA_AVAILABLE) {
      JoystickButton driveToBall = new JoystickButton(m_joystick, Constants.DRIVE_TO_BALL_BUTTON);
      driveToBall.whenPressed(new DriveTowardBall(m_driveSubsystem, m_joystick));
      driveToBall.whenReleased(new StopMotor(m_driveSubsystem));
    }


    if (Constants.TURRET_AVAILABLE) {
      JoystickButton shooter = new JoystickButton(m_joystick,Constants.SHOOT_BUTTON);
      shooter.whenPressed(new Shoot(magazine, turret, pickup));
      shooter.whenReleased(new ShootStop(magazine, turret));
      JoystickButton aimerCW = new JoystickButton(m_joystick,Constants.TURRETCW_BUTTON);
      shooter.whenPressed(new TurretCW(turret));
      shooter.whenReleased(new TurretStop(turret));
      JoystickButton aimerCCW = new JoystickButton(m_joystick,Constants.TURRETCCW_BUTTON);
      shooter.whenPressed(new TurretCCW(turret));
      shooter.whenReleased(new TurretStop(turret));
      JoystickButton aimerSpeed = new JoystickButton(m_joystick,Constants.TURRET_FINE_BUTTON);
      shooter.whenPressed(new TurretFine(turret, true));
      shooter.whenReleased(new TurretFine(turret, false));
    }
    
    // choosable autoCommands
    m_chooser = new SendableChooser<>();
    final Command m_autoDrive = 
      new AutoLeaveTarmacDistance(m_driveSubsystem);
    m_chooser.setDefaultOption("JustDrive", m_autoDrive);
    final Command m_autoGoToCargo = 
      new AutoDriveToCargo(m_driveSubsystem);
    m_chooser.addOption("Go to Cargo", m_autoGoToCargo);
    /*
    final Command m_autoShoot = 
      new AutoShoot(turret, magazine, pickup);
    m_chooser.addOption("Shoot", m_autoShoot);
    final Command m_autoShNGC = 
      new AutoShootAndGetCargo(turret, magazine, drive, pickup);
    m_chooser.addOption("Shoot and get Cargo", m_autoShNGC);
    final Command m_autoShootTwo = 
      new AutoShootTwo(turret, magazine, drive, pickup);
    m_chooser.addOption("Shoot Two", m_autoShootTwo);
    FIXME
    */
    SmartDashboard.putData(m_chooser);
    
    sideChooser = new SendableChooser<>();
    sideChooser.setDefaultOption("Right", true);
    sideChooser.addOption("Left", false);
    SmartDashboard.putData(sideChooser);
    
    // for testing purposes
    JoystickButton autocargoButton = new JoystickButton(m_joystick, 1);
    autocargoButton.whenPressed(new AutoDriveToCargo(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
  
}
