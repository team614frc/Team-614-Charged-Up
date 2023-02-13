package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrainSubsystem extends SubsystemBase {
  // Create Drivetrain Motor Variables
  CANSparkMax followerRightMotor = null;
  CANSparkMax leaderRightMotor = null;
  CANSparkMax followerLeftMotor = null;
  CANSparkMax leaderLeftMotor = null;

  // Create Differntial Drive Variables
  // Differential drive is used to call arcade drive using the motors.
  DifferentialDrive differentialDrive = null;

  public DriveTrainSubsystem() {
    // motor initalization
    followerRightMotor = new CANSparkMax(Constants.DRIVETRAIN_FOLLOWER_RIGHT_MOTOR, MotorType.kBrushless);
    leaderRightMotor = new CANSparkMax(Constants.DRIVETRAIN_LEADER_RIGHT_MOTOR, MotorType.kBrushless);
    followerLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_FOLLOWER_LEFT_MOTOR, MotorType.kBrushless);
    leaderLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_LEADER_LEFT_MOTOR, MotorType.kBrushless);

    // Leader motors follow follower motors and invertion is set.
    // Note: ROBOT MAY NOT GO STRAIGHT AND INVERTION MAY NEED TO CHANGE
    // MAKE SURE TO SET THE CURRENT LIMITS AS WELL
    // NOTE: LEADER MOTORS are LEADERS in this example
    followerRightMotor.follow(leaderRightMotor, false);
    followerLeftMotor.follow(leaderLeftMotor, false);

    // Current Limits Set
    followerRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    leaderRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    followerLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    leaderLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);

    // Create DifferentialDrive Object
    differentialDrive = new DifferentialDrive(leaderLeftMotor, leaderRightMotor);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Returns rate of motor
  public double getPosition() {
    double positionAverage = (Math.abs(leaderLeftMotor.getEncoder().getPosition())
        + Math.abs(leaderRightMotor.getEncoder().getPosition())) / 2;
    SmartDashboard.putNumber("Drivetrain Subsystem Encoder Position", positionAverage);
    return positionAverage;
  }

  public void setSpeed(double val) {
    leaderRightMotor.set(val);
    leaderLeftMotor.set(val);
    SmartDashboard.putNumber("Drive Left Motor Subsystem Speed Value ", leaderLeftMotor.get());
    SmartDashboard.putNumber("Drivetrain Right Motor Subsystem Speed Value", val);
  }

  public void resetEncoderValues() {
    leaderLeftMotor.getEncoder().setPosition(0.0);
    leaderRightMotor.getEncoder().setPosition(0.0);
  }

  public void rotateRight(double val) {
    leaderRightMotor.set(-val);
    leaderLeftMotor.set(val);
  }

  public void rotateLeft(double val) {
    leaderRightMotor.set(val);
    leaderLeftMotor.set(-val);
  }
}