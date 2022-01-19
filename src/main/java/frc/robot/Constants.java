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
        TURRET_SHOOTER       (00, false);

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

    // joystick and button numbers
    public static final int JOYSTICK = 0;
    public static final int SHIFTER_BUTTON = 2;
    public static final int DRIVE_TO_BALL_BUTTON = 3;
    public static final int DEPLOY_PICKUP_BUTTON = 6;
    public static final int RETRACT_PICKUP_BUTTON = 7;

    // motor speed constants
    public static final int PNEUMATICS_CONTROL_MODULE = 0;
    public static final double PICKUP_ROLLER_MOTOR_SPEED = .4;
    public static final double SHIFTER_THRESHOLD = 10.6;
    public static final double DRIVE_AIMER_SPEED_LIMIT = 0.4;
    public static final double SHOOTER_SPEED = 0;

    // Camera constants
    public static final int TURRET_PIXY_ANALOG = 0;
    public static final int CENTER_OF_CAMERA = 316/2; //hardwired for Pixy2 resolution
    public static final double PIXY_TARGET_AR = 2.;
    public static final int PIXY_MINIMUM_SIZE = 10;
}
