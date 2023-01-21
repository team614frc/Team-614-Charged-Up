// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.WPILibVersion;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // GLOBAL STOP MOTOR
    public static final int STOP_MOTOR = 0; 

    // GLOBAL INVERT
    public static final int GLOBAL_INVERT = -1; 

    // DRIVE TRAIN MOTORS
    public static final int DRIVETRAIN_TOP_RIGHT_MOTOR = 1;
    public static final int DRIVETRAIN_BOTTOM_RIGHT_MOTOR = 12;
    public static final int DRIVETRAIN_TOP_LEFT_MOTOR = 14;
    public static final int DRIVETRAIN_BOTTOM_LEFT_MOTOR = 15;
    public static final int MOTOR_CURRENT_LIMIT = 40;

    // Xbox Controller 
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Arcade Drive Commands
    public static final double ARCADE_DRIVE_MULTIPLIER = 0.5; 
    public static final int POW_VALUE = 3; 

    // Timer Based Auto Variables
    public static final double RUN_INITAL_AUTO = 2.0; 
    public static final double AUTO_STAGE_2 = 7.0;
    public static final double AUTO_STAGE_3 = 10.0;
    public static final double AUTO_STAGE_4 = 11.0;
    public static final double AUTO_DRIVE_SPEED = 0.5;
    public static final double AUTO_REVERSE_SPEED = -0.5;
    public static final double AUTO_ROTATE_SPEED = 0.0;
    public static final double AUTO_DRIVE_DISTANCE_INCHES = 36; //3 feet
    public static final double WAIT_TIME = 5;

    // Encoder Variables
    public static final int ENCODER_CPR = 1024;
    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (WHEEL_DIAMETER_INCHES * Math.PI) / (double) ENCODER_CPR; // Calculates circumference of wheels in inches

  public static class OperatorConstants {
   // public static final int kDriverControllerPort = 0;

  }
}
