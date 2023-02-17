package frc.robot.commands.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DrivePositionPIDCommand extends PIDCommand {

  @Override
  public void initialize() {
    RobotContainer.driveTrainSubsystem.resetEncoderValues();
  }

  public DrivePositionPIDCommand(double driveSetpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
        // This should return the measurement
        RobotContainer.driveTrainSubsystem::getEncoderPositionAverage,
        // This should return the setpoint (can also be a constant)
        driveSetpoint,
        // This uses the output
        RobotContainer.driveTrainSubsystem::setSpeed);
    SmartDashboard.putNumber("Position setpoint value", getController().getSetpoint());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
  }

  // Returns true when the encoders have reached the setpoint
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Position setpoint value", getController().getSetpoint());
    return getController().atSetpoint();
  }
}
