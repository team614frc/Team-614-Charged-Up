package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MaxTiltDown extends CommandBase {

  public MaxTiltDown() {
    addRequirements(RobotContainer.tiltSubsystem);
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.tiltSubsystem.getRightHeight() < Constants.TILT_MAX_ENCODER_VALUE ||
        RobotContainer.tiltSubsystem.getLeftHeight() < Constants.TILT_MAX_ENCODER_VALUE)) {
      RobotContainer.tiltSubsystem.set(Constants.TILT_DOWN_SPEED);
    } else {
      RobotContainer.tiltSubsystem.set(Constants.MOTOR_ZERO_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.tiltSubsystem.set(Constants.MOTOR_ZERO_SPEED);
  }

  // gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return false;
  }
}
