package frc.robot.commands.TimedCommands.TimedBasedAuto.TimedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TimedTurnDrive extends CommandBase {

  Timer arcadeDriveTimer = null;
  // Local variables for this command
  double localEndTime;
  double localRotation;

  public TimedTurnDrive(double rotation, double runtime) {
    // Requires the drivetrain subsystem for the command
    addRequirements(RobotContainer.driveTrainSubsystem);
    // Creates a new timmer
    arcadeDriveTimer = new Timer();
    localRotation = rotation;
    localEndTime = runtime;
  }

  @Override
  public void initialize() {
    // Resets and starts the timer
    arcadeDriveTimer.reset();
    arcadeDriveTimer.start();
  }

  @Override
  public void execute() {
    // Runs the motors for the specified runtime
    if (arcadeDriveTimer.get() <= localEndTime)
      RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.MOTOR_ZERO_SPEED, localRotation);
  }

  @Override
  public void end(boolean interrupted) {
    // Stops the motor, stops the timer, resets the timer, and if needed alerts the
    // drive if the command was interrupted
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.MOTOR_ZERO_SPEED, Constants.MOTOR_ZERO_SPEED);
    arcadeDriveTimer.stop();
    arcadeDriveTimer.reset();
    if (interrupted) {
      System.out.println("COMMAND WAS INTERRUPTED! : DIDN'T FINISH ON TIME!");
    }
  }

  @Override
  public boolean isFinished() {
    // Returns true when the command is finished
    return arcadeDriveTimer.get() >= localEndTime;
  }
}
