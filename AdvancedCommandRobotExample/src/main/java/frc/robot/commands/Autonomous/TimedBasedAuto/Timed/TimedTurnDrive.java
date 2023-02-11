// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.TimedBasedAuto.Timed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TimedTurnDrive extends CommandBase {
  /** Creates a new DriveForward. */
  Timer arcadeDriveTimer = null;
  double localEndTime;
  double localRotation;

  public TimedTurnDrive (double rotation, double runtime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);

    arcadeDriveTimer = new Timer();
    localRotation = rotation;
    localEndTime = runtime;
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
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.MOTOR_ZERO_SPEED, localRotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  //Stops the motor and timer, then resets the timer.
  public void end(boolean interrupted) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.MOTOR_ZERO_SPEED, Constants.MOTOR_ZERO_SPEED);
    arcadeDriveTimer.stop();
    arcadeDriveTimer.reset();
    if (interrupted)
    {
      System.out.println("COMMAND WAS INTERRUPTED! : DIDN'T FINISH ON TIME!");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arcadeDriveTimer.get() >= localEndTime;
  }
}
