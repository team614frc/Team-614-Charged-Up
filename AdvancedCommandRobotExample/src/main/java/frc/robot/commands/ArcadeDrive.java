package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
boolean isPressedDown = RobotContainer.m_CommandXboxController.povDown().getAsBoolean();
boolean isPressedUp = RobotContainer.m_CommandXboxController.povUp().getAsBoolean();
  public ArcadeDrive() {
    addRequirements(RobotContainer.driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Raw Speed
    double moveRawSpeed = RobotContainer.m_CommandXboxController.getLeftY();
    double rotateRawSpeed = RobotContainer.m_CommandXboxController.getRightX();

    // Adjusted Speed
    double moveAdjustedSpeed = Constants.ARCADE_DRIVE_MULTIPLIER * moveRawSpeed
        + Constants.ARCADE_DRIVE_MULTIPLIER * Math.pow(moveRawSpeed, Constants.POW_VALUE);
    double rotateAdjustedSpeed = Constants.ARCADE_DRIVE_MULTIPLIER * rotateRawSpeed
        + Constants.ARCADE_DRIVE_MULTIPLIER * Math.pow(rotateRawSpeed, Constants.POW_VALUE);
    // Uses an equation in order to get exact values for the amount value that we
    // are getting from the left sitck.

    // Arcade Drive
    // Right side of drivetrain is inverted
    // RobotContainer.driveTrainSubsystem.arcadeDrive(-moveAdjustedSpeed, -rotateAdjustedSpeed);
    // if(isPressedDown = true){
    //   RobotContainer.driveTrainSubsystem.arcadeDrive(-moveAdjustedSpeed, -rotateAdjustedSpeed*0.5);
    // }
    // else if (isPressedUp = true){
    // RobotContainer.driveTrainSubsystem.arcadeDrive(-moveAdjustedSpeed, -rotateAdjustedSpeed);
    // }
    // else {
      RobotContainer.driveTrainSubsystem.arcadeDrive(-moveAdjustedSpeed, -rotateAdjustedSpeed);
    // }
    // Passes the adjusted movement values and rotation values
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.MOTOR_ZERO_SPEED, Constants.MOTOR_ZERO_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
