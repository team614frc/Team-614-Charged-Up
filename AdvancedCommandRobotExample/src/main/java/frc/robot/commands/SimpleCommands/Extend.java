package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Extend extends CommandBase {

  public Extend() {
  addRequirements(RobotContainer.elevatorSubsystem);
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.elevatorSubsystem.getLeftHeight() < Constants.ELEVATOR_MAX_HEIGHT))  {
              RobotContainer.elevatorSubsystem.set(Constants.ELEVATOR_UP_SPEED);
            }
    else {
      RobotContainer.elevatorSubsystem.set(Constants.MOTOR_ZERO_SPEED);
    }
  }
 
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevatorSubsystem.set(Constants.MOTOR_ZERO_SPEED);
  }

  // gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return false;
  }

}
