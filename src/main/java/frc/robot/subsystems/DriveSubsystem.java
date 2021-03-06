// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pixy2API.Pixy2;
import frc.robot.Pixy2API.Pixy2CCC;
import frc.robot.Pixy2API.Pixy2CCC.Block;
import frc.robot.Pixy2API.links.I2CLink;

//import javax.print.CancelablePrintJob;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** DriveSubsystem
 *  two motors on each side
 *  two-speed gear-box
 *  Pixy camera via I2C
 * 
 *  Prefered mode is closed loop velocity control,
 * (See Constants.DRIVE_VELOCITY_CONTROLLED),
 *  but 'Vbus' mode is also available.
 * 
 *  Control inputs are smoothed to reduce jerkiness.
 */
public class DriveSubsystem extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax leftFollower;
  CANSparkMax rightMotor;
  CANSparkMax rightFollower;
  SparkMaxPIDController leftController;
  SparkMaxPIDController rightController;
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  double motor_ki=0.;
  double motor_kp=10.e-5;
  double motor_kd=0.;
  double motor_FF=.000015;
  Joystick joystick;
  PneumaticHub pcm;
  //PneumaticsControlModule pcm;
  DoubleSolenoid shifter;
  //DifferentialDrive driverControl;
  MyDifferentialDrive driverControl;
  double gearRatio = 1.;  // initialized as if there is no speed control
  double[] smoothTurn = new double[Constants.DRIVE_SMOOTHER_SAMPLES];
  double[] smoothStrait = new double[Constants.DRIVE_SMOOTHER_SAMPLES];
  double turnDriveStick;
  double straitDriveStick;
  int smoothIt = 0;

  Pixy2 driveCamera;
  double[] toNT = new double[10*5+2];  // at most 10 blocks of (sig, x, y, H, W) to be sent to NetworkTables
    
  /** Instantiate drive motor controllers and associated PID controller and encoder;
   *  pneumatic system,
   *  camera to use for displaying objects of interest and aiding the pickup of cargo
   */
  public DriveSubsystem(PneumaticHub pcm) {
    leftMotor = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_LEADER.getid(), MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_FOLLOWER.getid(), MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.CANIDs.DRIVE_RIGHT_LEADER.getid(), MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.CANIDs.DRIVE_RIGHT_FOLLOWER.getid(), MotorType.kBrushless);

   
    leftMotor.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftMotor.setInverted(Constants.CANIDs.DRIVE_LEFT_LEADER.isInverted());
    leftFollower.setInverted(Constants.CANIDs.DRIVE_LEFT_FOLLOWER.isInverted());

    rightMotor.setInverted(Constants.CANIDs.DRIVE_RIGHT_LEADER.isInverted());
    rightFollower.setInverted(Constants.CANIDs.DRIVE_RIGHT_FOLLOWER.isInverted());

    leftFollower.follow(leftMotor);
    rightFollower.follow(rightMotor);

    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();
    leftController.setP(motor_kp);
    leftController.setI(motor_ki);
    leftController.setD(motor_kd);
    leftController.setFF(motor_FF);
    rightController.setP(motor_kp);
    rightController.setI(motor_ki);
    rightController.setD(motor_kd);
    rightController.setFF(motor_FF);
    leftController.setOutputRange(-1, 1);
    rightController.setOutputRange(-1, 1);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    //driverControl = new DifferentialDrive(leftMotor, rightMotor);
    //pid controller ver.
    driverControl = new MyDifferentialDrive(leftMotor,rightMotor
    , leftController, rightController
    , leftEncoder, rightEncoder);
    

    if (Constants.COMPRESSOR_AVAILABLE ){
      this.pcm = pcm;
      shifter = pcm.makeDoubleSolenoid(Constants.PneumaticChannel.DRIVE_LOW_GEAR.getChannel(), Constants.PneumaticChannel.DRIVE_HIGH_GEAR.getChannel());
      switchGears(Constants.DRIVE_LOW_GEAR);
    }

    if (Constants.CAMERA_AVAILABLE) {
      driveCamera = Pixy2.createInstance(new I2CLink());
      int initError = driveCamera.init(Constants.PIXY_USE_MXP);
    }

    for (int i=0;i<Constants.DRIVE_SMOOTHER_SAMPLES;i++) {
      smoothTurn[i]=0.;
      smoothStrait[i]=0.;
    }
    smoothIt=0;
    turnDriveStick = 0.;
    straitDriveStick = 0.;
  }

  public void drive(double left, double right) {
    leftMotor.set(right);
    rightMotor.set(right);
  }

  /** DriveMe(right+, forward+) 
   * -1 < turnStick and straitStick < 1.
  */
  public void driveMe (double turnStick, double straitStick) {
    /* Speed limit enforced */
    double speed = turnStick*turnStick + straitStick*straitStick;
    if (speed > Constants.DRIVE_SPEED_LIMIT) {
      turnStick *= Constants.DRIVE_SPEED_LIMIT/Math.sqrt(speed);
      straitStick *= Constants.DRIVE_SPEED_LIMIT/Math.sqrt(speed);
    }
    if (Constants.AUTOSHIFT_AVAILABLE) {
      /* Alternative shifting strategies:
        A) Since the stick represents the robot speed,
          pick a robot speed to switch gears.
        B) Pick a motor speed at which to shift.
          Get the motor speeds from the encoder.
        C) In addition to either A or B, add information regarding
          how hard the motors are working to maintain speed,
          ie current draw or voltage.
      */
      if (speed > Constants.SHIFTER_THRESHOLD  ){
        switchGears(Constants.DRIVE_HIGH_GEAR);
      } else if (speed < Constants.SHIFTER_THRESHOLD*.95) {
        switchGears ( Constants.DRIVE_LOW_GEAR);
      }
    }

    // smooth the control inputs
    turnDriveStick += turnStick/Constants.DRIVE_SMOOTHER_SAMPLES - smoothTurn[smoothIt];
    straitDriveStick += straitStick/Constants.DRIVE_SMOOTHER_SAMPLES - smoothStrait[smoothIt];
    smoothTurn[smoothIt]=turnStick/Constants.DRIVE_SMOOTHER_SAMPLES;
    smoothStrait[smoothIt]=straitStick/Constants.DRIVE_SMOOTHER_SAMPLES;
    smoothIt = (smoothIt+1)%Constants.DRIVE_SMOOTHER_SAMPLES;

    //double IM_NOT_DONE_YET;
    //turnDriveStick=0.; straitDriveStick=.44;  // hardwire for testing

    //driverControl.arcadeDrive(-turnStick, straitStick);
    
    //SmartDashboard.putString("Gear: ",shifter.get()==Constants.DRIVE_HIGH_GEAR ?"High":"Low");
    double gearedTurn = turnDriveStick*gearRatio;
    double gearedStrait = straitDriveStick*gearRatio;
    //SmartDashboard.putNumber("to arcade: gearedStrait" , gearedStrait);
    //SmartDashboard.putNumber("to arcade: gearedTurn" , gearedTurn);
    //SmartDashboard.putNumber("to arcade: gearRatio ", gearRatio);
    driverControl.arcadeDrive(gearedStrait, gearedTurn);
  }
  

  /** stop the motors */
  public void stop(){
    driveMe(0.,0.);
  }

  /** toggle between two gears */
  public void switchGears() {
    //System.out.println("Switch gears");
    switchGears((shifter.get() == Constants.DRIVE_HIGH_GEAR)?Constants.DRIVE_LOW_GEAR:Constants.DRIVE_HIGH_GEAR);
  }

  public void switchGears(DoubleSolenoid.Value newGear) {
    //System.out.println("Switch gears "+newGear+" from "+shifter.get());
    if (Constants.COMPRESSOR_AVAILABLE){
      if (Constants.DRIVE_VELOCITY_CONTROLLED){
        if (newGear == Constants.DRIVE_HIGH_GEAR) {
          gearRatio = Constants.DRIVE_HIGH_GEAR_RATIO;
          shifter.set(Constants.DRIVE_HIGH_GEAR);
        } else if (newGear == Constants.DRIVE_LOW_GEAR
            && ! overRedLine()
          ) {
          gearRatio = Constants.DRIVE_LOW_GEAR_RATIO;
          shifter.set(Constants.DRIVE_LOW_GEAR);
        }
      } else {
        if (newGear == Constants.DRIVE_HIGH_GEAR) {
          shifter.set(Constants.DRIVE_HIGH_GEAR);
        } else if (newGear == Constants.DRIVE_LOW_GEAR) {
          shifter.set(Constants.DRIVE_LOW_GEAR);
        }
      }
    }
  }
  
  /** check whether speed is low enough to shift down
   *   without exceeding motor max RPM
   */
  private boolean overRedLine() {
    return 
       Math.abs(turnDriveStick) > Constants.SHIFTER_THRESHOLD
       ||  
       Math.abs(straitDriveStick) > Constants.SHIFTER_THRESHOLD;
  }

  /** get the encoder position (and, relatively, the robot position)
   * 
   * @return position in inches
   */
  public double getPosition() {
    return rightEncoder.getPosition() * (rightMotor.getInverted()?1.:-1.) /
     ((shifter.get() == Constants.DRIVE_LOW_GEAR)?Constants.DRIVE_DISTANCE_ENCODER_RATIO_LOW:Constants.DRIVE_DISTANCE_ENCODER_RATIO_HIGH);
  }

  @Override
  public void periodic() {
    int mySig = Constants.PIXY_SIG_BLUE;
    int offset=1;
    double biggest=0.;
    double size=0.;
    int numTargets;
    // This method will be called once per scheduler run
    // get pixy camera signatures
    // and send to NetWorkTables for display on dashboard
    if (DriverStation.getAlliance() == Alliance.valueOf("Blue")) {
      mySig = Constants.PIXY_SIG_BLUE;
    } else if (DriverStation.getAlliance() == Alliance.valueOf("Red")) {
      mySig = Constants.PIXY_SIG_RED;
    }
    if (Constants.CAMERA_AVAILABLE){
      numTargets = driveCamera.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL,10);
      toNT[0] = (double)numTargets;
      if (numTargets > 1){
        for (Block block : driveCamera.getCCC().getBlockCache())  {
          if (block.getWidth() > size) {
                toNT[offset+0] = block.getSignature();
                toNT[offset+1] = block.getX();
                toNT[offset+2] = block.getY();
                toNT[offset+3] = block.getWidth();
                toNT[offset+4] = block.getHeight();
                double tsize=block.getWidth()*block.getHeight();
                if (tsize>size && block.getSignature() == mySig) {
                  size=tsize;
                  biggest = (double)offset;
                }
                offset+=5;
          }
        }
      }
      toNT[offset]=biggest;
      SmartDashboard.putNumberArray("DriveCamera", toNT);
    }

  }

  /** report whether the camera sees anything */
  public int seeCargo() {
    if (Constants.CAMERA_AVAILABLE) {
      return (int)toNT[0];
    } else return 0;
  }

  /** report the x location (relative to the screen left)
   *  of the nearest cargo (determined by size)
   * Return -1 if none found
   */
  public double cargoX() {
    int number = (int)toNT[0];
    int biggest = (int)toNT[1+number*5];
    if (biggest > 0.) return toNT[biggest+1];
    return -1.;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
