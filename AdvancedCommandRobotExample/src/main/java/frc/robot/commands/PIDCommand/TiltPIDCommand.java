package frc.robot.commands.PIDCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TiltPIDCommand extends CommandBase {
  /** Creates a new TiltPIDCommand. */
  public double tiltSetpoint;

  public TiltPIDCommand(double tiltsetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.tiltSubsystem);
    tiltSetpoint = tiltsetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.tiltSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.tiltSubsystem.setSetpoint(tiltSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.tiltSubsystem.atSetpoint();
  }
}
