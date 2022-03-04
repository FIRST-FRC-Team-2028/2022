// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private PneumaticHub pcm;
  private Turret turret;
  private Pickup pickup;
  private Magazine magazine;
  private Joystick joystick;
  private Alliance alliance;
  private Thread turretUSBthread;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    pcm = m_robotContainer.getPcm();
    turret = m_robotContainer.getTurret();
    pickup = m_robotContainer.getPickup();
    magazine = m_robotContainer.getMagazine();
    joystick  = m_robotContainer.getJoystick();
    
    /* test quadraticFitter 
    QuadraticFitter fitter = new QuadraticFitter();
    double [][] testers = {
                          //{0, 1,    4,    9},
                          //{.5,1.7, 17.3, 83.3}    // a,b,c = 1, .2, .5
                          {5, 1,    4,    9},
                          {10.,23.6,14.6,-16.4}     // a,b,c =-.4, -1., 25.
                          };
    fitter.add(testers[0][0], testers[1][0]);
    fitter.add(testers[0][1], testers[1][1]);
    fitter.add(testers[0][2], testers[1][2]);
    fitter.add(testers[0][3], testers[1][3]);
    fitter.fitit();
    double [] tests = {0.,1.,2.,3.,4.,5.,6.,7.,8.,9};
    for (double each : tests)
      System.out.println(each+" , "+fitter.yout(each));
    */

    if (Constants.USBCAMERA_AVAILABLE){
      if (Constants.CAMERA_THREAD){
        turretUSBthread = 
          new Thread ( () -> {
            UsbCamera camera = CameraServer.startAutomaticCapture();
            //System.out.println("running camera thread");
            // Set the resolution
            camera.setResolution(640, 480);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();

            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();

            while (!Thread.interrupted()) {
              if (cvSink.grabFrame(mat) == 0) {
                outputStream.notifyError(cvSink.getError());
                continue;
              }
              //System.out.println("Add lines");
              // add a vertical line at center and three horizontal lines for range finding
              Imgproc.line(mat, new Point(320,100), new Point(320,400), new Scalar(0,0,0),5);
              Imgproc.line(mat, new Point(320-Constants.CAM_HUB_DIST.TARMAC.getW(),Constants.CAM_HUB_DIST.TARMAC.getH()),
                                new Point(320+Constants.CAM_HUB_DIST.TARMAC.getW(),Constants.CAM_HUB_DIST.TARMAC.getH()), 
                                new Scalar(255,0,0), 5);
              Imgproc.line(mat, new Point(320-Constants.CAM_HUB_DIST.CARGO_RING.getW(),Constants.CAM_HUB_DIST.CARGO_RING.getH()),
                                new Point(320+Constants.CAM_HUB_DIST.CARGO_RING.getW(),Constants.CAM_HUB_DIST.CARGO_RING.getH()), 
                                new Scalar(255,125,0), 5);
              Imgproc.line(mat, new Point(320-Constants.CAM_HUB_DIST.PAD1.getW(),Constants.CAM_HUB_DIST.PAD1.getH()),
                                new Point(320+Constants.CAM_HUB_DIST.PAD1.getW(),Constants.CAM_HUB_DIST.PAD1.getH()), 
                                new Scalar(0,0,0), 5);
              outputStream.putFrame(mat);
            }
          });
        turretUSBthread.setDaemon(true);
        turretUSBthread.start();
      }else
        CameraServer.startAutomaticCapture();
    }

    alliance = DriverStation.getAlliance();
  }

  public Alliance getAlliance(){
    return alliance;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (Constants.COMPRESSOR_AVAILABLE ){
      //pcm.enableCompressorDigital();
      pcm.enableCompressorAnalog(110.,120.);
      double pressure= pcm.getPressure(Constants.COMPRESSOR_ANALOG_CHANNEL);
      SmartDashboard.putNumber("Pressure", pressure);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  enum TestModes {
    SHOOTER(0),
    ELEVATOR(1),
    TURRET(2),
    PICKUP_ROLLERS(3),
    PICKUP_ARMS(4),
    MAG_HORI(5),
    MAG_VERT(6),
    CLIMB_EXTEND(7),
    CLIMBER(8),
    TURRET_TURRRET(9);

    int id;
    TestModes(final int id){
      this.id=id;
    }
    int getID() {return id;}
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    joystick  = new Joystick(Constants.JOYSTICK);
    elevationAngle = 45.;
    elevationUp = new JoystickButton(joystick,Constants.ELEVATOR_UP_BUTTON);
    elevationDown = new JoystickButton(joystick,Constants.ELEVATOR_DOWN_BUTTON);
  }

  private double elevationAngle;
  private JoystickButton elevationUp;
  private JoystickButton elevationDown;

  boolean startedTesting=false;
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    double speed;
    if (Constants.JOYSTICK_EXTREME3D)
      speed = joystick.getRawAxis(3);
    else
      speed = joystick.getZ();
    //System.out.println("test A");
    int testState = TestModes.MAG_VERT.getID();
    
    if(testState == TestModes.SHOOTER.getID()) {   // check that stooter rollers toss ball out
      speed=(speed+1.)/2.*5700.;  // 0 < speed < 5700
      turret.shooterSpeed(speed);
      SmartDashboard.putNumber("shooterSpeed", speed);

    } else if (testState == TestModes.ELEVATOR.getID()) {  // check that positive is elevator up/forward
      turret.elevatorMove(speed*1000.);
      SmartDashboard.putNumber("elevationAngle", turret.getElevation());

    } else if (testState == TestModes.TURRET_TURRRET.getID()) {  //  turret azimuth +CW
      turret.turretCW(speed);
    } else if (testState == TestModes.PICKUP_ROLLERS.getID()) {  //  pickup rollers
      // initialize the running average
      if (!startedTesting) {
        pickup.initAvg();
        startedTesting = true;
      }
      pickup.avg_update();
      speed=(speed+1.)/2.;  // 0 < speed < 1
      pickup.runRollers(speed);
      if (joystick.getRawButtonPressed(11))pickup.deploy();
      if (joystick.getRawButtonPressed(10))pickup.retract();
      // smartdash data available after disable
      SmartDashboard.putNumber("pickupSpeed", speed);
      SmartDashboard.putNumber("pickupRPM", pickup.getRollerVelocity());
      /**Dont understand sendable registry */
      //SendableRegistry.setName(pickup.getrollers(), "Pickup", "Rollers");
      if(pickup.hasCargo()) {
        System.out.println("pickup has" + pickup.numCargo());
      }

    } else if (testState == TestModes.MAG_HORI.getID()) {  //  magazine

    } else if (testState == TestModes.MAG_VERT.getID()) {  //  magazine
      System.out.println("test B");
      if (joystick.getRawButtonPressed(11))magazine.verticalon();
      if (joystick.getRawButtonPressed(10))magazine.verticaloff();
      speed=(speed+1.)/2.*5700.;  // 0 < speed < 5700
      turret.shooterSpeed(speed);
      System.out.println(speed);
      SmartDashboard.putNumber("shooterSpeed", speed);
    }
  }
}
