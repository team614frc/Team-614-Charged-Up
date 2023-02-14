package frc.robot.commands.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RotateRightPIDCommand extends PIDCommand {

  double setpoint;

  public RotateRightPIDCommand(double rotationSetpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
        // This should return the measurement
        RobotContainer.driveTrainSubsystem::getEncoderPositionAverage,
        // This should return the setpoint (can also be a constant)
        rotationSetpoint,
        // This uses the output)
        RobotContainer.driveTrainSubsystem::rotateRight);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
