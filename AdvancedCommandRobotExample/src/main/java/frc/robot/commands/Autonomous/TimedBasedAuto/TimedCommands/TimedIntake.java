package frc.robot.commands.Autonomous.TimedBasedAuto.TimedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class TimedIntake extends CommandBase {

  Timer TimedIntakeTimer = null;
  // Local variables for this command
  double localIntakeSpeed;
  double localRunTime;

  public TimedIntake(double intakespeed, double runtime) {
    // Requires the drivetrain subsystem for the command
    addRequirements(RobotContainer.manipulator);
    // Creates a new timmer
    TimedIntakeTimer = new Timer();

    localIntakeSpeed = intakespeed;
    localRunTime = runtime;
  }

  @Override
  public void initialize() {
    // Resets and starts the timer
    TimedIntakeTimer.reset();
    TimedIntakeTimer.start();
  }

  @Override
  public void execute() {
    // Runs the motors for the specified runtime
    if (TimedIntakeTimer.get() <= localRunTime)
      RobotContainer.manipulator.set(localIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    // Stops the motor, stops the timer, resets the timer, and if needed alerts the
    // drive if the command was interrupted
    RobotContainer.manipulator.set(Constants.MOTOR_ZERO_SPEED);
    TimedIntakeTimer.stop();
    TimedIntakeTimer.reset();
    if (interrupted) {
      System.out.println("COMMAND WAS INTERRUPTED! : DIDN'T FINISH ON TIME!");
    }
  }

  @Override
  public boolean isFinished() {
    // Returns true when the command is finished
    return TimedIntakeTimer.get() >= localRunTime;
  }
}
