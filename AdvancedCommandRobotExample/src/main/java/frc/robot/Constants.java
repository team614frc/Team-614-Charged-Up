// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // GLOBAL STOP MOTOR
  public static final double STOP_MOTOR_SPEED = 0.0;

  // GLOBAL INVERT
  public static final int GLOBAL_INVERT = -1;

  // DRIVE TRAIN MOTORS
  public static final int DRIVETRAIN_FRONT_RIGHT_MOTOR = 1;
  public static final int DRIVETRAIN_BACK_RIGHT_MOTOR = 3;
  public static final int DRIVETRAIN_FRONT_LEFT_MOTOR = 14;
  public static final int DRIVETRAIN_BACK_LEFT_MOTOR = 15;
  public static final int MOTOR_CURRENT_LIMIT = 40;

  // INTAKE MOTORS
  public static final int INTAKE_MOTOR = 12;

  // Xbox Controller
  public static final int DRIVER_CONTROLLER_PORT = 0;

  // Arcade Drive Commands
  public static final double ARCADE_DRIVE_MULTIPLIER = 0.5;
  public static final int POW_VALUE = 3;

  // Manipulator Commands
  public static final double INTAKE_SPEED_FORWARD = 1;
  public static final double INTAKE_SPEED_BACKWARD = -1;

  // Elevator Commands
  public static final double ELEVATOR_UP_SPEED = 0.5;
  public static final double ELEVATOR_DOWN_SPEED = -0.5;
  public static final int ELEVATOR_CURRENT_LIMIT = 40;

  // Elevator PID Setpoints
  public static final double ELEVATOR_SETPOINT = 10;
  public static final double ELEVATOR_SETPOINT2 = 0;

  // Manipulator PID setpoints (test)
  public static final double MANIPULATOR_SETPOINT = 25; // for testing
  public static final double MANIPULATOR_SETPOINT2 = -10;

  // Thresholds
  public static final double MANIPULATOR_THRESHOLD = 10;

  // Tilt Commands
  public static final double TILT_UP_SPEED = 0.5;
  public static final double TILT_DOWN_SPEED = -0.5;

  // PID Values
  public static final double kP = 0.01;
  public static final double kI = 0;
  public static final double kD = 0;

  // Timer Based Auto Variables
  public static final double RUN_INITAL_AUTO = 2.0;
  public static final double AUTO_STAGE_2 = 7.0;
  public static final double AUTO_STAGE_3 = 10.0;
  public static final double AUTO_STAGE_4 = 11.0;
  public static final double AUTO_DRIVE_SPEED = 0.5;
  public static final double AUTO_REVERSE_SPEED = -0.5;
  public static final double AUTO_ROTATE_SPEED = 0.0;

  // ELEVATOR MOTOR ID'S
  public static final int ELEVATOR_RIGHT_MOTOR = 2;
  public static final int ELEVATOR_LEFT_MOTOR = 13;

  // TILT MOTOR ID'S
  public static final int TILT_RIGHT_MOTOR = 22;
  public static final int TILT_LEFT_MOTOR = 23;

  public static class OperatorConstants {
    // public static final int kDriverControllerPort = 0;

  }
}
