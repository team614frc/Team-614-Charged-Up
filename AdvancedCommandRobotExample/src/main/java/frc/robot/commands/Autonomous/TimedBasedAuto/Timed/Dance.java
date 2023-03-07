package frc.robot.commands.Autonomous.TimedBasedAuto.Timed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Dance extends CommandBase {
  private final double DANCE_TIME = 5.0; // dance time in seconds
  private final double ROTATION_SPEED = 0.5; // rotation speed of the robot
  private final double ROTATION_TIME = 0.5; // time to rotate in one direction
  
  private double endTime; // time when the dance should end
  private boolean isRotatingLeft; // whether the robot is currently rotating left
  
  public Dance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTime = Timer.getFPGATimestamp() + DANCE_TIME;
        isRotatingLeft = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime < endTime) {
        if (isRotatingLeft) {
            RobotContainer.driveTrainSubsystem.arcadeDrive(0.0, ROTATION_SPEED);
        } else {
          RobotContainer.driveTrainSubsystem.arcadeDrive(0.0, -ROTATION_SPEED);
        }
        if (currentTime % ROTATION_TIME < 0.01) {
            isRotatingLeft = !isRotatingLeft;
        }
    } else {
      RobotContainer.driveTrainSubsystem.arcadeDrive(0.0, 0); // stop the robot at the end of the dance
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}