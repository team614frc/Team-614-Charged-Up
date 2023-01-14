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

  // Create Motor Variables
  CANSparkMax topRightMotor = null;
  CANSparkMax bottomRightMotor = null;
  CANSparkMax topLeftMotor = null;
  CANSparkMax bottomLeftMotor = null;

  // Create Differntial Drive Variables
  DifferentialDrive differentialDrive = null;

  //FOR SIMULATION PURPOSES ONLY 
  private final Field2d m_field = new Field2d();
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(),0,0);



  public DriveTrainSubsystem() {

    // motor initalization
    CANSparkMax topRightMotor = new CANSparkMax(Constants.DRIVETRAIN_TOP_RIGHT_MOTOR, MotorType.kBrushless);
    CANSparkMax bottomRightMotor = new CANSparkMax(Constants.DRIVETRAIN_BOTTOM_RIGHT_MOTOR, MotorType.kBrushless);
    CANSparkMax topLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_TOP_LEFT_MOTOR, MotorType.kBrushless);
    CANSparkMax bottomLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_BOTTOM_LEFT_MOTOR, MotorType.kBrushless);

    // Bottom motors follow top motors and invertion is set. 
    // Note: ROBOT MAY NOT GO STRAIGHT AND INVERTION MAY NEED TO CHANGE
    // MAKE SURE TO SET THE CURRENT LIMITS AS WELL
    // NOTE: BOTTOM MOTORS are LEADERS in this example 
    topRightMotor.follow(bottomRightMotor,false);
    topLeftMotor.follow(bottomLeftMotor,false);

    // Current Limits Set
    topRightMotor.setSmartCurrentLimit(40);
    bottomRightMotor.setSmartCurrentLimit(40);
    topLeftMotor.setSmartCurrentLimit(40);
    bottomLeftMotor.setSmartCurrentLimit(40);

    // Create Differential Drive Object 
    DifferentialDrive differentialDrive  = new DifferentialDrive(bottomLeftMotor, bottomRightMotor);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  private Pose2d getPose2d()
  {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(null, 0, 0)
    m_field.setRobotPose(getPose2d());
  }
}
