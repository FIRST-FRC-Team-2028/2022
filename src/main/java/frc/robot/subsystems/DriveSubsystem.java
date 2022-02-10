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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  Joystick joystick;
  PneumaticHub pcm;
  //PneumaticsControlModule pcm;
  DoubleSolenoid shifter;
  //DifferentialDrive driverControl;
  MyDifferentialDrive driverControl;
  double gearRatio = 1.;  // initialized as if there is no speed control
  double[] smoothX = new double[Constants.DRIVE_SMOOTHER_SAMPLES];
  double[] smoothY = new double[Constants.DRIVE_SMOOTHER_SAMPLES];
  double xDriveStick;
  double yDriveStick;
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
    rightController.setP(motor_kp);
    rightController.setI(motor_ki);
    rightController.setD(motor_kd);
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
      smoothX[i]=0.;
      smoothY[i]=0.;
    }
    smoothIt=0;
    xDriveStick = 0.;
    yDriveStick = 0.;
  }

  public void drive(double left, double right) {
    leftMotor.set(right);
    rightMotor.set(right);
  }

  /** DriveMe(right+, forward+) 
   * -1 < stickX and stickY < 1.
  */
  public void driveMe (double stickX, double stickY) {
    /* Speed limit enforced */
    double speed = stickX*stickX + stickY*stickY;
    if (speed > Constants.DRIVE_SPEED_LIMIT) {
      stickX *= Constants.DRIVE_SPEED_LIMIT/Math.sqrt(speed);
      stickY *= Constants.DRIVE_SPEED_LIMIT/Math.sqrt(speed);
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
    xDriveStick += stickX/Constants.DRIVE_SMOOTHER_SAMPLES - smoothX[smoothIt];
    yDriveStick += stickY/Constants.DRIVE_SMOOTHER_SAMPLES - smoothY[smoothIt];
    smoothX[smoothIt]=stickX/Constants.DRIVE_SMOOTHER_SAMPLES;
    smoothY[smoothIt]=stickY/Constants.DRIVE_SMOOTHER_SAMPLES;
    smoothIt = (smoothIt+1)%Constants.DRIVE_SMOOTHER_SAMPLES;

    double IM_NOT_DONE_YET;
    xDriveStick=0.; yDriveStick=.44;  // hardwire for testing
    /* Saturday, end of day, getting unexplained behavior:
      shift from low to high on the stand, ie without load,
      seems to nearly maintain wheel speed - for a couple seconds.
      After that brief period, the wheel speed abruptly increases.
      Shifting down slows back down again. The behavior repeats
      whenever you shift. When the robot was on the floor, presumable under load,
      the time to switch speed after gear change was undetectable,
      so it appeared the high gear ratio was wrong.
    */

    //driverControl.arcadeDrive(-stickX, stickY);
    
    SmartDashboard.putString("Gear: ",shifter.get()==Constants.DRIVE_HIGH_GEAR ?"High":"Low");
    double xGearedStick = -xDriveStick*gearRatio;
    double yGearedStick = yDriveStick*gearRatio;
    SmartDashboard.putNumber("to arcade: xGearedStick" , xGearedStick);
    SmartDashboard.putNumber("to arcade: yGearedStick" , yGearedStick);
    SmartDashboard.putNumber("to arcade: gearRatio ", gearRatio);
    driverControl.arcadeDrive(xGearedStick, yGearedStick);
  }
  

  /** stop the motors */
  public void stop(){
    driveMe(0.,0.);
  }

  /** toggle between two gears */
  public void switchGears() {
    System.out.println("Switch gears");
    switchGears((shifter.get() == Constants.DRIVE_HIGH_GEAR)?Constants.DRIVE_LOW_GEAR:Constants.DRIVE_HIGH_GEAR);
  }

  public void switchGears(DoubleSolenoid.Value newGear) {
    System.out.println("Switch gears "+newGear+" from "+shifter.get());
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
       Math.abs(xDriveStick) > Constants.SHIFTER_THRESHOLD
       ||  
       Math.abs(yDriveStick) > Constants.SHIFTER_THRESHOLD;
  }

  @Override
  public void periodic() {
    int offset=1;
    double biggest=0.;
    double size=0.;
    int numTargets;
    // This method will be called once per scheduler run
    // get pixy camera signatures
    // and send to NetWorkTables for display on dashboard
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
                if (tsize>size) {
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

  public int seeCargo() {
    if (Constants.CAMERA_AVAILABLE) {
      return (int)toNT[0];
    } else return 0;
  }

  public double cargoX() {
    int number = (int)toNT[0];
    int biggest = (int)toNT[1+number*5];
    return toNT[biggest+1];
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
