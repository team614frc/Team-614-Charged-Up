// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    CANSparkMax frontRightMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    CANSparkMax backRightMotor = new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_MOTOR, MotorType.kBrushless);
    CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_MOTOR, MotorType.kBrushless);
    CANSparkMax backLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_MOTOR, MotorType.kBrushless);

    // Bottom motors follow top motors and invertion is set. 
    // Note: ROBOT MAY NOT GO STRAIGHT AND INVERTION MAY NEED TO CHANGE
    // MAKE SURE TO SET THE CURRENT LIMITS AS WELL
    // NOTE: BOTTOM MOTORS are LEADERS in this example 
    frontRightMotor.follow(backRightMotor,false);
    frontLeftMotor.follow(backLeftMotor,false);

    // Current Limits Set
    frontRightMotor.setSmartCurrentLimit(40);
    backRightMotor.setSmartCurrentLimit(40);
    frontLeftMotor.setSmartCurrentLimit(40);
    backLeftMotor.setSmartCurrentLimit(40);

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
