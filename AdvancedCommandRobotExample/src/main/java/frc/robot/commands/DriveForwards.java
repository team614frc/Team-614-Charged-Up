// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveForwards extends CommandBase {
  /** Creates a new DriveForward. */
  public DriveForwards () {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives the motors forward
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.AUTO_DRIVE_SPEED, Constants.AUTO_ROTATE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
