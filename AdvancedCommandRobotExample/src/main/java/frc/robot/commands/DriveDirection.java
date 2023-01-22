// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveDirection extends CommandBase {
  /** Creates a new DriveForward. */
  Timer arcadeDriveTimer = null; 
  double localSpeed; 
  double localRotation;
  double localEndTime;

  public DriveDirection (double speed, double rotation, double endtime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);

    arcadeDriveTimer = new Timer();
    localSpeed = speed;
    localRotation = rotation;
    localEndTime = endtime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arcadeDriveTimer.reset();
    arcadeDriveTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives the motors forward
    if (arcadeDriveTimer.get() <= localEndTime) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(localSpeed, localRotation);
    }
   // RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.STOP_MOTOR, Constants.STOP_MOTOR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.STOP_MOTOR, Constants.STOP_MOTOR);
    arcadeDriveTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
