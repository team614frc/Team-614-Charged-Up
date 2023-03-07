package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
  // Tick Conversions
  public static final double M_PI = Math.PI;
  public static final double ksVolts = .34281;
  public static final double kvVoltSecondsPerMeter = 1.316;
  public static final double kaVoltSecondsSquaredPerMeter = 0.14549;
  public static final double kTrackWidthMeters = Units.inchesToMeters(20);
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  public static final double kGearRatio = 10.71; // need to change to 8.45 with new robot
  public static final double kWheelRadiusInches = 3;
  public static final double kLinearDistanceConversionFactor = (Units
      .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));

  // PWM LED Port
  public static final int ADDRESSABLE_LED_PORT = 0;

  // GLOBAL STOP MOTOR
  public static final double MOTOR_ZERO_SPEED = 0.0;
  public static final double MOTOR_REST_BACK = 0.05;

  // GLOBAL INVERT
  public static final int GLOBAL_INVERT = -1;

  // DRIVE TRAIN MOTORS
  public static final int DRIVETRAIN_FOLLOWER_RIGHT_MOTOR = 9;
  public static final int DRIVETRAIN_LEADER_RIGHT_MOTOR = 2;
  public static final int DRIVETRAIN_FOLLOWER_LEFT_MOTOR = 10;
  public static final int DRIVETRAIN_LEADER_LEFT_MOTOR = 19;
  public static final int MOTOR_CURRENT_LIMIT = 40;

  // INTAKE MOTORS
  public static final int INTAKE_MOTOR = 4; // 4

  // Xbox Controller
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int CO_DRIVER_CONTROLLER_PORT = 1;

  // Xbox Controller Buttons
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;
  public static final int A_BUTTON = 1;
  public static final int B_BUTTON = 2;
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
  public static final double ELEVATOR_UP_SPEED = 0.8;
  public static final double ELEVATOR_DOWN_SPEED = -0.8;
  public static final int ELEVATOR_CURRENT_LIMIT = 40;

  // Elevator PID Setpoints
  public static final double ELEVATOR_SETPOINT = 0;
  public static final double ELEVATOR_SETPOINT2 = 5;

  // Elevator Min and Max height
  public static final double ELEVATOR_MAX_HEIGHT = 34; // 22.8
  public static final double ELEVATOR_MIN_HEIGHT = 5;

  // Tilt Min and Max height
  public static final double TILT_MIN_ENCODER_VALUE = 2; // Pivot going downwards increases encoder value
  public static final double TILT_MAX_ENCODER_VALUE = 20; // 17 //15

  // Manipulator PID setpoints (test)
  public static final double MANIPULATOR_SPEED_INTAKE = 0.8; // for testing
  public static final double MANIPULATOR_SPEED_OUTTAKE = -0.8;
  public static final double MANIPULATOR_SPEED_BLEH = -0.4;
  public static final double MANIPULATOR_SPEED_PCHOO = -1;

  // Thresholds
  public static final double MANIPULATOR_THRESHOLD = 10;

  // Tilt Commands
  public static final double TILT_UP_SPEED = 0.4;
  public static final double TILT_DOWN_SPEED = -0.2;
  public static final double TILT_REST_SPEED = 0.05;
  public static final double TILT_DEFAULT_SETPOINT = 5;
  public static final double TILT_LOAD_STATION_SETPOINT = 10.48; //10.48 for charge station
  public static final double TILT_HIGH_CUBE_AUTO_SETPOINT = 13;
  public static final double TILT_MID_SCORE_SETPOINT = 14.5;
  public static final double TILT_PCHOO_SETPOINT = 18;
  public static final double TILT_LOW_SETPOINT = 22;
  public static final double TILT_UP_SETPOINT = 0.5;
  public static final double TILT_HIGH_CUBE_SETPOINT = 13;

  // Position PID Values
  public static final double P_kP = 0.023; // 1.18
  public static final double P_kI = 0.00001; // 0.06
  public static final double P_kD = 0; // 1.95
  // Pivot PID Values
  public static final double Pivot_kP = 0.025; // 1.18
  public static final double Pivot_kI = 0; // 0.06
  public static final double Pivot_kD = 0; // 1.95

  // Elevator PID Values
  public static final double Elevator_kP = 0.000000001; // 0.08
  public static final double Elevator_kI = 0; // 0.01
  public static final double Elevator_kD = 0; // 6.14

  // Velocity-based PID Values
  public static final double V_kP = 0.433933;
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
  public static final int ELEVATOR_RIGHT_MOTOR = 17;
  public static final int ELEVATOR_LEFT_MOTOR = 3;

  // TILT MOTOR ID'S
  public static final int TILT_RIGHT_MOTOR = 18;
  public static final int TILT_LEFT_MOTOR = 1;

  public static class OperatorConstants {
  }
}