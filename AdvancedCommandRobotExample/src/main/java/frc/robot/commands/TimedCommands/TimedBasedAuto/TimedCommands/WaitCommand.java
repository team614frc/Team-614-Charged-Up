package frc.robot.Commands.TimedCommands.TimedBasedAuto.TimedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class WaitCommand extends CommandBase {

  Timer WaitTimer = null;
  // Local variables for this command
  double localRunTime;
  /*
   * A command which will stop the drivetrain
   * for a specified amount of time.
   */

  public WaitCommand(double runtime) {
    // Requires the drivetrain subsystem for the command
    addRequirements(RobotContainer.driveTrainSubsystem);
    // Creates a new timmer
    WaitTimer = new Timer();

    localRunTime = runtime;
  }

  @Override
  public void initialize() {
    // Resets and starts the timer
    WaitTimer.reset();
    WaitTimer.start();
  }

  @Override
  public void execute() {
    // Runs the motors for the specified runtime
    if (WaitTimer.get() <= localRunTime)
      RobotContainer.driveTrainSubsystem.setSpeed(Constants.MOTOR_ZERO_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    // Stops the motor, stops the timer, resets the timer, and if needed alerts the
    // drive if the command was interrupted
    RobotContainer.driveTrainSubsystem.setSpeed(Constants.MOTOR_ZERO_SPEED);
    WaitTimer.stop();
    WaitTimer.reset();
    if (interrupted) {
      System.out.println("COMMAND WAS INTERRUPTED! : DIDN'T FINISH ON TIME!");
    }
  }

  @Override
  public boolean isFinished() {
    // Returns true when the command is finished
    return WaitTimer.get() >= localRunTime;
  }
}
