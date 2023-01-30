package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class Tilt extends CommandBase{
    public double tiltSpeed;
    public Tilt (double tiltspeed) {
        addRequirements(RobotContainer.elevatorSubsystem);
        tiltSpeed = tiltspeed;
    }
    @Override
  public void initialize() {
    
  }

  public void test(){

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.elevatorSubsystem.set(tiltSpeed);
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevatorSubsystem.set(Constants.MOTOR_ZERO_SPEED);
  }
  //gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return false;
  }
}
