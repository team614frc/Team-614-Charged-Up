package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class TimedIntake extends CommandBase{

Timer TimedIntakeTimer = null;
public double intakeSpeed;

double localRunTime;

    public TimedIntake(double intakespeed, double runtime){
        addRequirements(RobotContainer.manipulator);

        TimedIntakeTimer = new Timer();

        intakeSpeed = intakespeed;
        localRunTime = runtime;
    }
  
    @Override
  public void initialize() {
    TimedIntakeTimer.reset();
    TimedIntakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TimedIntakeTimer.get() <= localRunTime)
    RobotContainer.manipulator.set(intakeSpeed);
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.manipulator.set(Constants.STOP_MOTOR);
    TimedIntakeTimer.stop();
    TimedIntakeTimer.reset();
  }
  //gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return TimedIntakeTimer.get() <= localRunTime;
  }
}
