package frc.robot.commands.Autonomous.TimedBasedAuto.Timed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class TimedExtend extends CommandBase {
public double elevationSpeed;

Timer TimedExtendTimer = null;
double localRunTime;


    public TimedExtend (double elevationspeed, double runtime) {
        addRequirements(RobotContainer.elevatorSubsystem);

        TimedExtendTimer = new Timer();

        elevationSpeed = elevationspeed;
        localRunTime = runtime;
    }

    @Override
  public void initialize() {
    TimedExtendTimer.reset();
    TimedExtendTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TimedExtendTimer.get() <= localRunTime)
    RobotContainer.elevatorSubsystem.set(elevationSpeed);
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevatorSubsystem.set(Constants.MOTOR_ZERO_SPEED);
    TimedExtendTimer.stop();
    TimedExtendTimer.reset();
  }
  //gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return TimedExtendTimer.get() <= localRunTime;
  }
}
