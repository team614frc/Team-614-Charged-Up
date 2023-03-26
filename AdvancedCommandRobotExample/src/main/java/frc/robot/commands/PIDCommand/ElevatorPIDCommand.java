package frc.robot.commands.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ElevatorPIDCommand extends PIDCommand {
  // Makes 3 setpoints for elevator subsystem to check if at certain height
  public ElevatorPIDCommand(double elevatorSetpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.Elevator_kP, Constants.Elevator_kI, Constants.Elevator_kD),
        // This should return the measurement
        RobotContainer.elevatorSubsystem::getRightHeight,
        // This should return the setpoint (can also be a constant)
        elevatorSetpoint,
        // This uses the output
        RobotContainer.elevatorSubsystem::set);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
