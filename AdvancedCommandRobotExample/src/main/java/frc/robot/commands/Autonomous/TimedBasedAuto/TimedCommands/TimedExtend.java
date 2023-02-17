package frc.robot.commands.Autonomous.TimedBasedAuto.TimedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class TimedExtend extends CommandBase {

  Timer TimedExtendTimer = null;
  // Local variables for this command
  double localElevationSpeed;
  double localRunTime;

  public TimedExtend(double elevationspeed, double runtime) {
    // Requires the drivetrain subsystem for the command
    addRequirements(RobotContainer.elevatorSubsystem);
    // Creates a new timmer
    TimedExtendTimer = new Timer();

    localElevationSpeed = elevationspeed;
    localRunTime = runtime;
  }

  @Override
  public void initialize() {
    // Resets and starts the timer
    TimedExtendTimer.reset();
    TimedExtendTimer.start();
  }

  @Override
  public void execute() {
    // Runs the motors for the specified runtime
    if (TimedExtendTimer.get() <= localRunTime)
      RobotContainer.elevatorSubsystem.set(localElevationSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    // Stops the motor, stops the timer, resets the timer, and if needed alerts the
    // drive if the command was interrupted
    RobotContainer.elevatorSubsystem.set(Constants.MOTOR_ZERO_SPEED);
    TimedExtendTimer.stop();
    TimedExtendTimer.reset();
    if (interrupted) {
      System.out.println("COMMAND WAS INTERRUPTED! : DIDN'T FINISH ON TIME!");
    }
  }

  @Override
  public boolean isFinished() {
    // Returns true when the command is finished
    return TimedExtendTimer.get() >= localRunTime;
  }
}
