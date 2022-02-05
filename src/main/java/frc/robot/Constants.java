// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /*
     * Flags that tell the code systems exist
     */
    public static final boolean DRIVE_AVAILABLE         = true;
    public static final boolean AUTOSHIFT_AVAILABLE     = false;
    public static final boolean CAMERA_AVAILABLE        = false;
    public static final boolean AIM_AVAILABLE           = false;
    public static final boolean TURRET_AVAILABLE        = false;
    public static final boolean MAGAZINE_AVAILABLE      = false;
    public static final boolean PICKUP_AVAILABLE        = false;
    public static final boolean KICKER_AVAILABLE        = false;
    public static final boolean CLIMBER_AVAILABLE       = false;
    public static final boolean CONTROLPANEL_AVAILABLE  = false;
    public static final boolean COMPRESSOR_AVAILABLE    = true;
    public static final boolean GYRO_AVAILABLE          = false;
    public static final boolean BUTTONBOX_AVAILABLE     = false;

    // CAN connections
    public enum CANIDs {
        DRIVE_LEFT_LEADER    (20, true), 
        DRIVE_LEFT_FOLLOWER  (21, true),
        DRIVE_RIGHT_LEADER   (10, true), 
        DRIVE_RIGHT_FOLLOWER (11, true),
        PICKUP_ROLLERS       (40, true),
        TURRET_ELEVATION     (00, false),
        TURRET_AZIMUTH       (00, false),
        TURRET_SHOOTER       (00, false),
        MAGIZINE_HORIZONTAL  (00, false), 
        MAGIZINE_VERTICAL    (00, false);

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
        DRIVE_LOW_GEAR(0),
        DRIVE_HIGH_GEAR(1),
        PICKUP_RETRACT(2),
        PICKUP_EXTEND(3),
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

    // joystick and button numbers
    public static final int JOYSTICK = 0;
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

    // motor speed constants
    public static final double PICKUP_ROLLER_MOTOR_SPEED = .4;
    public static final double DRIVE_AIMER_SPEED_LIMIT = 0.4;
    public static final double MAGAZINE_VERTICAL_MOTOR_SPEED = 0;
    public static final double MAGAZINE_HORIZONTAL_MOTOR_SPEED = 0;
    public static final double SHOOTER_SPEED = 0;
    public static final double SHOOTER_SLOW_SPEED = 0;

    // Camera constants
    public static final int TURRET_PIXY_ANALOG = 0;
    public static final int PIXY_USE_MXP = 0;
    public static final int CENTER_OF_CAMERA = 316/2; //hardwired for Pixy2 resolution
    public static final double PIXY_TARGET_AR = 2.;
    public static final int PIXY_MINIMUM_SIZE = 10;

    // Turret constants
    public static final double SHOOTER_FCN_ACOEF = 0;
    public static final double SHOOTER_FCN_BCOEF = 0;
    public static final double SHOOTER_FCN_CCOEF = 0;
    public static final int ELEVATOR_MOTOR_ENCODER_RATIO = 0;
    public static final double ELEVATOR_MOTOR_ZEROING_SPEED = 0;
    public static final double SPARKMAX_RPM = 0;
    public static final double TARMAC_DISTANCE = 0;
    public static final double PAD_ONE_DISTANCE =0;
    public static final double PAD_TWO_DISTANCE =0;


    // drive constants
    /* Max RPM = 5700 rpm
        high gear ratio = 2.667
        wheel radius = 6"
        Max speed = 5700 /2.667 *6*PI /12/5280 *60 = 37.5 mph
     */
    public static final double DRIVE_HIGH_GEAR_RATIO = 3000;   //  Highest desired Motor RPM
    public static final double DRIVE_LOW_GEAR_RATIO = 2.2727*DRIVE_HIGH_GEAR_RATIO;  // high:low ratio of gear box
    public static final DoubleSolenoid.Value DRIVE_LOW_GEAR = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DRIVE_HIGH_GEAR = DoubleSolenoid.Value.kReverse;
    public static final double SHIFTER_THRESHOLD = 0.6;
    
    // climber constants
    public static final double CLIMBER_TIME_TO_CLIMB = 0;
    

}
