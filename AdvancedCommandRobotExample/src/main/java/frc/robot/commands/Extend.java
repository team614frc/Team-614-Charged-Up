package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;


    //Elevator code by Zabir and Danial

public class Extend extends CommandBase {
public double elevationSpeed;

    public Extend (double y) {
        addRequirements(RobotContainer.elevator);
        elevationSpeed = y;
    }

    @Override
  public void initialize() {
    
  }

  public void test(){

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.elevator.set(elevationSpeed);
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevator.set(Constants.STOP_MOTOR_SPEED);
  }
  //gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
