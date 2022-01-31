// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.TreeUI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Flag that tells the code systems exist
     */
    public static final boolean DRIVE_AVAILABLE         = true;
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

    public enum CANIDs {
        DRIVE_LEFT_LEADER    (20, true), 
        DRIVE_LEFT_FOLLOWER  (21, true),
        DRIVE_RIGHT_LEADER   (10, true), 
        DRIVE_RIGHT_FOLLOWER (11, true),
        PICKUP_ROLLERS       (40, true),
        TURRET_ELEVATION     (00, false),
        TURRET_AZIMUTH       (00, false),
        TURRET_SHOOTER       (00, false),
        MAGIZINE_HORIZONTAL (00, false), 
        MAGIZINE_VERTICAL   (00, false);

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

    /** Enum to hold all information about pneumatic solenoids */
    public enum PneumaticChannel {
        DRIVE_LOW_GEAR(0),
        DRIVE_HIGH_GEAR(1),
        PICKUP_RETRACT(2),
        PICKUP_EXTEND(3),
        //OPEN_ACCELERATOR_FLAPS(4),
        //CLOSE_ACCELERATOR_FLAPS(5);
        CLIMBER_RELEASE(4);
        private final int channel;
        private PneumaticChannel(final int ch) {
            channel = ch;
        }
        public int getChannel() {
            return channel;
        }
    }

    
    public static final int PNEUMATICS_CONTROL_MODULE = 0;
   
    public static final int PIXY_USE_MXP = 1;  // Use MXP I2C port for Pixy2 camera
    public static final double CENTER_OF_CAMERA = 316/2; //Hardwired pixy2 resolution

    /** these constants are related to drivesubststem */
    public static final double SHIFTER_THRESHOLD = 0.6;

    /**these constants are related to pickup */
    public static final double ROLLER_MOTOR_SPEED = .4;

    /**these constants are related to buttons */
    public static final int JOYSTICK = 0;
    public static final int RETRACT_PICKUP_BUTTON = 7;
    public static final int DEPLOY_PICKUP_BUTTON = 6;
    public static final int SHIFTER_BUTTON = 2;
    public static final int SHOOT_BUTTON = 0;
    public static final int DEPLOY_CLIMBER_BUTTON = 0;

    /** these constants are related to magazine */
    public static final double MAGIZINE_HORIZONTAL_MOTOR_SPEED = 0;
    public static final double MAGIZINE_VERTICAL_MOTOR_SPEED = 0;
    

    /**These constans are related to the turret */
  
    public static final double SHOOTER_SLOW_SPEED = 0.2;
    public static final double SHOOTER_SPEED = 0;
    public static final int ELEVATOR_MOTOR_ENCODER_RATIO = 0;
    public static final int TURRET_PIXY_ANALOG = 0;
    
    /**these constants are related to the climber */
    public static final double CLIMBER_TIME_TO_CLIMB = 0;

    /** Field geometry constants */

    public static final double DISTANCE_FROM_HUB_TO_LAUNCH_PAD_1 =0.;
    public static final double DISTANCE_FROM_HUB_TO_LAUNCH_PAD_2 = 0.;
    public static final double DISTANCE_FROM_HUB_TO_TARMAC = 0.;

}
