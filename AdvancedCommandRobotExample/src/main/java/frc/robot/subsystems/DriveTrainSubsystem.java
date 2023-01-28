// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  // Create Drivetrain Motor Variables
  CANSparkMax frontRightMotor = null;
  CANSparkMax backRightMotor = null;
  CANSparkMax frontLeftMotor = null;
  CANSparkMax backLeftMotor = null;

  // Create Differntial Drive Variables
  // Differential drive is used to call arcade drive using the motors. 
  DifferentialDrive differentialDrive = null;

  public DriveTrainSubsystem() {

    // motor initalization
    frontRightMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR, MotorType.kBrushless);
    frontLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_MOTOR, MotorType.kBrushless);

    // Back motors follow front motors and invertion is set. 
    // Note: ROBOT MAY NOT GO STRAIGHT AND INVERTION MAY NEED TO CHANGE
    // MAKE SURE TO SET THE CURRENT LIMITS AS WELL
    // NOTE: BACK MOTORS are LEADERS in this example 
    frontRightMotor.follow(backRightMotor,false);
    frontLeftMotor.follow(backLeftMotor,false);

    // Current Limits Set
    frontRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    backRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    frontLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    backLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);

    // Create DifferentialDrive Object 
    differentialDrive = new DifferentialDrive(backLeftMotor, backRightMotor);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
