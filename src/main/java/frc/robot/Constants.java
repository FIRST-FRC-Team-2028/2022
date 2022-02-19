// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide constants.
 *  Do not put anything functional in this class.
 *
 * Subsystem availability
 * CAN ids
 * Pneumatic channels
 * joystick and button numbers
 * Camera
 * turret
 * drive
 * pickup
 * magazine
 * climber
 * field measurements
 * robot dimensions
 * 
 */
public final class Constants {

    /*
     * Flags that tell the code systems exist
     */
    public static final boolean DRIVE_AVAILABLE         = true;
    public static final boolean AUTOSHIFT_AVAILABLE     = true;
    public static final boolean CAMERA_AVAILABLE        = false;
    public static final boolean AIM_AVAILABLE           = false;
    public static final boolean TURRET_AVAILABLE        = false;
    public static final boolean MAGAZINE_AVAILABLE      = false;
    public static final boolean PICKUP_AVAILABLE        = true;
    public static final boolean KICKER_AVAILABLE        = false;
    public static final boolean CLIMBER_AVAILABLE       = false;
    public static final boolean CONTROLPANEL_AVAILABLE  = false;
    public static final boolean COMPRESSOR_AVAILABLE    = true;
    public static final boolean GYRO_AVAILABLE          = false;
    public static final boolean BUTTONBOX_AVAILABLE     = false;
    public static final boolean USBCAMERA_AVAILABLE     = false;
    public static final boolean JOYSTICK_EXTREME3D = true;

    // CAN connections
    public enum CANIDs {
        DRIVE_LEFT_LEADER    (20, false), 
        DRIVE_LEFT_FOLLOWER  (21, false),
        DRIVE_RIGHT_LEADER   (10, true), 
        DRIVE_RIGHT_FOLLOWER (11, true),
        PICKUP_ROLLERS       (60, true),
        TURRET_ELEVATION     (00, false),
        TURRET_AZIMUTH       (00, false),
        TURRET_SHOOTER       (00, false),
        MAGIZINE_HORIZONTAL  (00, false), 
        MAGIZINE_VERTICAL    (00, false), 
        CLIMB_MOTOR          (00,false);

        private final int canid;
        private final boolean inverted;

        CANIDs(final int id, final boolean inverted) {
            this.canid = id;
            this.inverted = inverted;
        }

        public int getid() {
            return canid;
        }

        public boolean isInverted() {
            return inverted;
        }
    }

    /** information about pneumatic solenoids */
    public enum PneumaticChannel {
        DRIVE_LOW_GEAR(1),
        DRIVE_HIGH_GEAR(0),
        PICKUP_RETRACT(3),
        PICKUP_EXTEND(2),
        CLIMBER_RELEASE(4);
        private final int channel;
        private PneumaticChannel(final int ch) {
            channel = ch;
        }
        public int getChannel() {
            return channel;
        }
    }
    public static final int PNEUMATICS_CONTROL_MODULE = 1;  // CAN id for PH
    public static final int COMPRESSOR_ANALOG_CHANNEL = 0;

    // joystick and button numbers
    public static final int JOYSTICK = 0;
    public static final int LEFT_BUTTON_BOX = 1;
    public static final int RIGHT_BUTTON_BOX = 2;
    public static final int SHIFTER_BUTTON = 2;
    public static final int SHIFTERU_BUTTON = 5;
    public static final int SHIFTERD_BUTTON = 4;
    public static final int DRIVE_TO_BALL_BUTTON = 3;
    public static final int DEPLOY_PICKUP_BUTTON = 6;
    public static final int RETRACT_PICKUP_BUTTON = 7;
    public static final int SHOOT_BUTTON = 0;
    public static final int TARMAC_DISTANCE_BUTTON = 0;
    public static final int PAD_TWO_DISTANCE_BUTTON = 0;
    public static final int PAD_ONE_DISTANCE_BUTTON = 0;
    public static final int ELEVATOR_UP_BUTTON = 0;
    public static final int ELEVATOR_DOWN_BUTTON = 0;
    public static final int TURRETCW_BUTTON = 0;
    public static final int TURRETCCW_BUTTON = 0;

    /* Camera constants ,
    from measurements half of horizontal field of view = 43, 
    half of vertical field of view = 27.4 degree
     */
    public static final int CENTER_OF_CAMERA = 316/2; //hardwired for Pixy2 x-resolution
    public static final int PIXY_VERT_CENTER =200/2;
    public static final int PIXY_FOV_VERT = 40;  // degrees 
    public static final int TURRET_PIXY_ANALOG = 0;
    public static final int TURRET_PIXY_ADDRESS = 0x48;
    public static final int DRIVE_PIXY_ADDRESS = 0x54;
    public static final int PIXY_USE_MXP = 0;
    public static final double PIXY_TARGET_AR = 2.;
    public static final int PIXY_MINIMUM_SIZE = 10;
    public static final double TURRET_AIMER_FILTER_SIZE = 0;
    public static final int PIXY_SIG_BLUE = 1;
    public static final int PIXY_SIG_RED = 2;
    public static final int PIXY_SIG_HUB = 3;
    public static final double PIXY_TAN_HORIZ_FOV = 47.25/50.5;
    public static final double PIXY_TAN_VERT_FOV  = 47.25/91.;
    public static final double CAM_HEIGHT = 0; //FIXME

    // Turret constants
    /* NEO 550 for turret and elevator
     *    11000 max RPM
     *    42 encoder counts/revolution
     *    gear ratio for elevation = 100 from gear box
     *    gear ratio to elevator = 18 : 42
     *    gear ratio for azimuth = 100 from gear box
     *    gear ratio to turret = N : M
     * 
     * NEO 1650 for shooter
     *    5700 max RPM
     */
    public static final double SHOOTER_FCN_ACOEF = 0.;
    public static final double SHOOTER_FCN_BCOEF = 0.;
    public static final double SHOOTER_FCN_CCOEF = 0.;
    public static final double ELEVATOR_MOTOR_ZEROING_SPEED = 0.; // better be negative
    public static final double ELEVATOR_FCN_ACOEF = 0.;
    public static final double ELEVATOR_FCN_BCOEF = 0.;
    public static final double ELEVATOR_FCN_CCOEF = 0.;
    public static final double ELEVATOR_ENCODER_RATIO =  42. / 18. /100.;
    public static final double TURRET_CAMERA_HEIGHT = 36.; //inches
    public static final double TURRET_TIME_TO_SHOOT = 0.;
    public static final int TURRET_SWITCH_CHANNEL = 0;
    public static final double SHOOTER_SPEED = 0.;  // 0 -> 1
    public static final double SHOOTER_SLOW_SPEED = 0.;  // 0 -> 1
    public static final double TURRET_MOTOR_SPEED = 0;  // 0 -> 1
    public static final double SHOOT_INDICATOR = 0;  // delta rpm


    // drive constants
    /* Max RPM = 5700 rpm
       wheel diameter = 6" 

        high gear ratio = 3.667
        Max speed = 5700 /3.667 *3*PI /12/5280 *60 = 13.8 mph = 20.2 ft/sec
        drive distance per motor revolution  Rd = 3 * 2PI / 3.667 = 5.14 in

        low gear ratio = 8.333
        max speed in low = 6.1 mph = 8.95 ft/sec
        drive distance per motor revolution  Rd = 3 * 2PI / 8.333 = 2.26 in

        NEO 1650 encoder:rotations = 42
     */
    public static final double SPARKMAX_RPM = 5700;
    public static final double DRIVE_SPEED_LIMIT = 0.6;
    public static final double DRIVE_HIGH_GEAR_RATIO = 1.;   //  Highest desired Motor RPM
    //public static final double DRIVE_LOW_GEAR_RATIO = 8.333/3.667*DRIVE_HIGH_GEAR_RATIO;  // high:low ratio of gear box
    //public static final double SHIFTER_THRESHOLD = 12.2/27.7;
    //  So much for theory; what really happens in practice the ratio is 2.1
    public static final double DRIVE_LOW_GEAR_RATIO = 2.1*DRIVE_HIGH_GEAR_RATIO;  // high:low ratio of gear box
    public static final double SHIFTER_THRESHOLD = 0.476;
    public static final DoubleSolenoid.Value DRIVE_LOW_GEAR = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DRIVE_HIGH_GEAR = DoubleSolenoid.Value.kReverse;
    public static final double DRIVE_LEAVE_TARMAC_SPEED = 0.2;
    public static final double DRIVE_LEAVE_TARMAC_DISTANCE = 0.;    // inches
    public static final double DRIVE_TIME_TO_LEAVE_TARMAC = 0.;
    public static final int DRIVE_SMOOTHER_SAMPLES = 10;
    public static final boolean DRIVE_VELOCITY_CONTROLLED = true;
    //public static final double DRIVE_DISTANCE_ENCODER_RATIO_LOW = 42.0/2.26;  //  per inch
    public static final double DRIVE_DISTANCE_ENCODER_RATIO_LOW = 1.0/2.26;  //  per inch
    public static final double DRIVE_DISTANCE_ENCODER_RATIO_HIGH = 1./5.14;  //  per inch
    public static final double DRIVE_AIMER_SPEED_LIMIT = 0.4;
    

    // pickup constants
    public static final int PICKUP_CARGO_INDICATION = 200;  //RPM difference
    public static final double PICKUP_ROLLER_MOTOR_SPEED = .6;

    //magazine constants
    public static final double MAGAZINE_VERTICAL_MOTOR_SPEED = 0.;
    public static final double MAGAZINE_HORIZONTAL_MOTOR_SPEED = 0.;

    // climber constants
    public static final double CLIMBER_TIME_TO_CLIMB = 0.;

    // field measurements
    public static final double HUB_HEIGHT = 104.; //inches
    public static final double TARMAC_DISTANCE = 0.;  // inches from Hub
    public static final double PAD_ONE_DISTANCE =0.;
    public static final double PAD_TWO_DISTANCE =0.;
    public static final double FIELD_TARMAC_TO_CARGO = 40.44 - 0.;   // inches
    public static final double FIELD_HUB_TO_PAD1 = 202.95;  // inches
    public static final double FIELD_HUB_TO_PAD2 = 244.77;  // inches

    // Robot measurements
    public static final double ROBOT_LENGTH = 38.;  // inches
    public static final double ROBOT_WIDTH = 0.;  // inches
    
}
