// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.TiltSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TiltSubsystem extends SubsystemBase {
  /** Creates a new TiltSubsystem. */
  CANSparkMax elevatorTopRightMotor = null;
  CANSparkMax elevatorTopLeftMotor = null;

  public TiltSubsystem() {
  elevatorTopRightMotor = new CANSparkMax(Constants.TILT_RIGHT_MOTOR, MotorType.kBrushless);
  elevatorTopLeftMotor = new CANSparkMax(Constants.TILT_LEFT_MOTOR, MotorType.kBrushless);

  elevatorTopRightMotor.follow(elevatorTopLeftMotor);

  elevatorTopRightMotor.setSmartCurrentLimit(40);
  elevatorTopLeftMotor.setSmartCurrentLimit(40);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(double val){
    elevatorTopLeftMotor.set(val);
  }
}
