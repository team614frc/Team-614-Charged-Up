package frc.robot.commands.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TiltPID extends PIDCommand {
  /** Creates a new TiltPID. */
  static double tiltSetpoint;
  public TiltPID(double tiltSetpoint) { 
    super(
        // The controller that the command will use
        new PIDController(Constants.Pivot_kP, Constants.Pivot_kI, Constants.Pivot_kD),
        // This should return the measurement
        RobotContainer.tiltSubsystem::getRightHeight,
        // This should return the setpoint (can also be a constant)
        tiltSetpoint,
        // This uses the output
        output -> {
          // Use the output here
          RobotContainer.tiltSubsystem.set(-output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.tiltSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }
//Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
