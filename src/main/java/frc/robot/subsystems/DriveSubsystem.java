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
 */
public class DriveSubsystem extends SubsystemBase {
  CANSparkMax leftMotor;
  CANSparkMax leftFollower;
  CANSparkMax rightMotor;
  CANSparkMax rightFollower;
  SparkMaxPIDController leftController;
  SparkMaxPIDController rightController;
  double motor_ki=0.;
  double motor_kp=.2;
  double motor_kd=0.;
  Joystick joystick;
  PneumaticHub pcm;
  //PneumaticsControlModule pcm;
  DoubleSolenoid shifter;
  DifferentialDrive driverControl;
  double gearRatio;
  //MyDifferentialDrive driverControl;
  Pixy2 driveCamera;
  double[] toNT = new double[10*5+2];  // at most 10 blocks of (sig, x, y, H, W)
    
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
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

    driverControl = new DifferentialDrive(leftMotor, rightMotor);
    //pid controller ver.
    /* 
    driverControl = new MyDifferentialDrive(leftMotor,rightMotor, leftController, rightController);
     */

    if (Constants.COMPRESSOR_AVAILABLE ){
      pcm = new PneumaticHub(Constants.PNEUMATICS_CONTROL_MODULE);
      //pcm = new PneumaticsControlModule(Constants.PNEUMATICS_CONTROL_MODULE);

      shifter = pcm.makeDoubleSolenoid(Constants.PneumaticChannel.DRIVE_LOW_GEAR.getChannel(), Constants.PneumaticChannel.DRIVE_HIGH_GEAR.getChannel());
      shifter.set(Constants.DRIVE_LOW_GEAR);
    }

    if (Constants.CAMERA_AVAILABLE) {
      driveCamera = Pixy2.createInstance(new I2CLink());
      int initError = driveCamera.init(Constants.PIXY_USE_MXP);
    }

  }

  public void drive(double left, double right) {
    leftMotor.set(right);
    //leftFollower.setVoltage(left);
    rightMotor.set(right);
    //rightFollower.setVoltage(right);
  }

  /** DriveMe(forward+, right+) 
   * -1 < stickX and stickY < 1.
  */
  public void driveMe (double stickX, double stickY) {
    if (Constants.AUTOSHIFT_AVAILABLE) {
      if (stickX*stickX+stickY*stickY > Constants.SHIFTER_THRESHOLD  ){
        switchGears(Constants.DRIVE_HIGH_GEAR);
      } else if (stickX*stickX+stickY*stickY < Constants.SHIFTER_THRESHOLD*.95) {
        switchGears ( Constants.DRIVE_LOW_GEAR);
      }
    }
    driverControl.arcadeDrive(-stickX, stickY);
    /*
    double leftMotorSpeed = stickX*gearRatio;
    double rightMotorSpeed = stickY*gearRatio;
    driverControl.arcadeDrive(leftMotorSpeed, rightMotorSpeed);
    */
  }
  

  /** stop the motors */
  public void stop(){
    driveMe(0.,0.);
  }

  /** toggle between two gears */
  public void switchGears() {
    System.out.println("Switch gears");
    if (Constants.COMPRESSOR_AVAILABLE){
      if (shifter.get() == Constants.DRIVE_LOW_GEAR) {
        gearRatio = Constants.DRIVE_HIGH_GEAR_RATIO;
        shifter.set(Constants.DRIVE_HIGH_GEAR);
      } else {
        gearRatio = Constants.DRIVE_LOW_GEAR_RATIO;
        shifter.set(Constants.DRIVE_LOW_GEAR);
      }
    }
  }

  public void switchGears(DoubleSolenoid.Value newGear) {
    System.out.println("Switch gears "+newGear);
    if (newGear == Constants.DRIVE_HIGH_GEAR 
        && shifter.get() ==  Constants.DRIVE_LOW_GEAR ) {
          System.out.println(" from "+shifter.get());
      gearRatio = Constants.DRIVE_HIGH_GEAR_RATIO;
      shifter.set(Constants.DRIVE_HIGH_GEAR);
    } else if (newGear == Constants.DRIVE_LOW_GEAR 
       && shifter.get() == Constants.DRIVE_HIGH_GEAR) {
        System.out.println(" from "+shifter.get());
      gearRatio = Constants.DRIVE_LOW_GEAR_RATIO;
      shifter.set(Constants.DRIVE_LOW_GEAR);
    }
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
      toNT[0]=(double)numTargets;
      if (numTargets >1){
        for (Block block : driveCamera.getCCC().getBlockCache())  {
          if (block.getWidth() > size) {
                toNT[offset+0]=block.getSignature();
                toNT[offset+1]=block.getX();
                toNT[offset+2]=block.getY();
                toNT[offset+3]=block.getWidth();
                toNT[offset+4]=block.getHeight();
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
    pcm.enableCompressorDigital();
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
