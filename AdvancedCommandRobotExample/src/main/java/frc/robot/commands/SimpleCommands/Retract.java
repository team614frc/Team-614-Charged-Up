package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Retract extends CommandBase {

  public Retract() {
  addRequirements(RobotContainer.elevatorSubsystem);
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotContainer.elevatorSubsystem.getLeftHeight() > Constants.ELEVATOR_MIN_HEIGHT))  {
              RobotContainer.elevatorSubsystem.set(Constants.ELEVATOR_DOWN_SPEED);
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
