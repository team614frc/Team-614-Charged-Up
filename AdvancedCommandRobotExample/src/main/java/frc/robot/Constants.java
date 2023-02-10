// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  // GLOBAL STOP MOTOR
  public static final double MOTOR_ZERO_SPEED = 0.0;
  public static final double MOTOR_REST_BACK = -0.05;

  // GLOBAL INVERT
  public static final int GLOBAL_INVERT = -1;

  // DRIVE TRAIN MOTORS
  public static final int DRIVETRAIN_FRONT_RIGHT_MOTOR = 1;
  public static final int DRIVETRAIN_BACK_RIGHT_MOTOR = 3;
  public static final int DRIVETRAIN_FRONT_LEFT_MOTOR = 14;
  public static final int DRIVETRAIN_BACK_LEFT_MOTOR = 13;
  public static final int MOTOR_CURRENT_LIMIT = 40;

  // INTAKE MOTORS
  public static final int INTAKE_MOTOR = 12; 

  // Xbox Controller
  public static final int DRIVER_CONTROLLER_PORT = 0;

  // Xbox Controller Buttons
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int BACK_BUTTON = 7;
  public static final int START_BUTTON = 8;
  public static final int LEFT_STICK_PRESS = 9;
  public static final int RIGHT_STICK_PRESS = 10;
  
  
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
  public static final double MANIPULATOR_SETPOINT = -30; // for testing
  public static final double MANIPULATOR_SETPOINT2 = 25;

  // Thresholds
  public static final double MANIPULATOR_THRESHOLD = 5;

  // Tilt Commands
  public static final double TILT_UP_SPEED = 0.5;
  public static final double TILT_DOWN_SPEED = -0.5;
  public static final double TILT_DOWN_SETPOINT = 0;
  public static final double TILT_UP_SETPOINT = 5;

  // Position-based PID Values
  public static final double P_kP = 0.01;
  public static final double P_kI = 0;
  public static final double P_kD = 0;

  // Velocity-based PID Values
  public static final double V_kP = 0.16375;
  public static final double V_kI = 0;
  public static final double V_kD = 0;

  // Timer Based Auto Variables
  public static final double RUN_INITAL_AUTO = 2.0;
  public static final double AUTO_STAGE_2 = 7.0;
  public static final double AUTO_STAGE_3 = 10.0;
  public static final double AUTO_STAGE_4 = 11.0;
  public static final double AUTO_DRIVE_SPEED = 0.5;
  public static final double AUTO_REVERSE_SPEED = -0.5;
  public static final double AUTO_ROTATE_SPEED = 0.0;

  // ELEVATOR MOTOR ID'S
  public static final int ELEVATOR_RIGHT_MOTOR = 50; //2
  public static final int ELEVATOR_LEFT_MOTOR = 54; //13

  // TILT MOTOR ID'S
  public static final int TILT_RIGHT_MOTOR = 51;
  public static final int TILT_LEFT_MOTOR = 2;

  public static class OperatorConstants {
    // public static final int kDriverControllerPort = 0;

  }
}
